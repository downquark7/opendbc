"""
Closed-loop consistency test for the blended LTA/LKAS prototype (TSS2 Corolla).

Drives the real CarController through engagement, LTA precision control, a handoff to
LKAS under high torque demand, a handback to LTA, and disengagement, while feeding every
generated steering message into the compiled panda safety model configured with the
LTA_BLEND param. The controller must never emit a message the safety model blocks, in
any phase or during any handoff.
"""
import os
import unittest

os.environ["TOYOTA_LTA_LKAS_BLEND"] = "1"

from opendbc.car import structs, DT_CTRL
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import CAR, ToyotaFlags, ToyotaSafetyFlags
from opendbc.car.toyota.carcontroller import BlendPhase
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety

EPS_SCALE = 73
V_EGO = 10.0  # m/s


class TestLtaLkasBlendClosedLoop(unittest.TestCase):
  def setUp(self):
    CarInterface = interfaces[CAR.TOYOTA_COROLLA_TSS2]
    fingerprints = {i: {} for i in range(7)}
    self.CP = CarInterface.get_params(CAR.TOYOTA_COROLLA_TSS2, fingerprints, [],
                                      alpha_long=False, is_release=False, docs=False)
    assert self.CP.flags & ToyotaFlags.LTA_LKAS_BLEND.value
    assert self.CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.LTA_BLEND.value

    # keep the scenario focused on lateral control
    self.CP.openpilotLongitudinalControl = False

    self.CI = CarInterface(self.CP)
    self.controller = self.CI.CC

    self.packer = CANPackerSafety("toyota_nodsu_pt_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota,
                                 EPS_SCALE | ToyotaSafetyFlags.LTA_BLEND)
    self.safety.init_tests()

    self.eps_torque = 0.0  # simulated EPS-applied torque
    self.frame = 0

  def _cs_stub(self, lat_active: bool):
    class CSStub:
      pass

    CS = CSStub()
    CS.out = structs.CarState()
    CS.out.vEgo = V_EGO
    CS.out.vEgoRaw = V_EGO
    CS.out.steeringTorqueEps = float(self.eps_torque)
    CS.out.steeringTorque = 0.0
    CS.out.steeringRateDeg = 0.0
    CS.out.steeringAngleDeg = 0.0
    CS.out.steeringAngleOffsetDeg = 0.0
    CS.acc_type = 1
    CS.pcm_follow_distance = 0
    CS.lkas_hud = {}
    CS.gvc = 0.0
    CS.secoc_synchronization = None
    return CS

  def _cc(self, lat_active: bool, torque: float, angle_deg: float):
    CC = structs.CarControl()
    CC.enabled = lat_active
    CC.latActive = lat_active
    CC.longActive = False
    CC.actuators.torque = torque
    CC.actuators.steeringAngleDeg = angle_deg
    CC.orientationNED = []
    CC.angularVelocity = []
    return CC.as_reader()

  def _rx_car_state(self):
    """Mirror the simulated car state into the safety model."""
    values = {("WHEEL_SPEED_%s" % n): V_EGO * 3.6 for n in ["FR", "FL", "RR", "RL"]}
    self.assertTrue(self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values)))

    values = {
      "STEER_TORQUE_EPS": (self.eps_torque / EPS_SCALE) * 100.,
      "STEER_TORQUE_DRIVER": 0,
      "STEER_ANGLE": 0,
      "STEER_ANGLE_INITIALIZING": 0,
    }
    self.assertTrue(self.safety.safety_rx_hook(self.packer.make_can_msg_safety("STEER_TORQUE_SENSOR", 0, values)))

  def _step(self, lat_active: bool, torque: float, angle_deg: float = 0.0):
    """Run one 100Hz control frame and pass all steering output through the safety model."""
    # advance the safety RT timer with real time: 10 ms per frame
    self.safety.set_timer(self.frame * 10000)
    self._rx_car_state()

    CS = self._cs_stub(lat_active)
    CC = self._cc(lat_active, torque, angle_deg)
    _, can_sends = self.controller.update(CC, CS, self.frame * int(DT_CTRL * 1e9))

    for addr, dat, bus in can_sends:
      if addr in (0x2E4, 0x191):
        msg = libsafety_py.make_CANPacket(addr, bus, dat)
        err = f"safety blocked addr {hex(addr)} in phase {self.controller.blend_phase} at frame {self.frame}: {dat.hex()}"
        self.assertTrue(self.safety.safety_tx_hook(msg), err)

    # crude EPS model: in LTA it applies its own torque; the LKAS interface it follows directly
    phase = self.controller.blend_phase
    if phase == BlendPhase.LTA and lat_active:
      self.eps_torque = min(self.eps_torque + 15, 200)  # small torque for fine corrections
    elif phase in (BlendPhase.LTA_WINDDOWN, BlendPhase.LTA_RELEASE):
      self.eps_torque = max(self.eps_torque - 15, 0)  # TORQUE_WIND_DOWN=0 ramp, ~1500 units/s
    else:
      self.eps_torque = float(self.controller.last_torque)

    self.frame += 1

  def test_full_scenario(self):
    # not engaged: everything must be no-actuation and accepted
    for _ in range(20):
      self._step(lat_active=False, torque=0.0)
    self.assertEqual(self.controller.blend_phase, BlendPhase.LTA)

    # engage
    self.safety.set_controls_allowed(True)

    # precision regime: low demand stays on LTA
    for _ in range(150):
      self._step(lat_active=True, torque=0.1, angle_deg=0.5)
    self.assertEqual(self.controller.blend_phase, BlendPhase.LTA)

    # high demand: hands off to LKAS through wind-down and release
    seen_phases = set()
    for _ in range(300):
      self._step(lat_active=True, torque=0.6, angle_deg=2.0)
      seen_phases.add(self.controller.blend_phase)
    self.assertEqual(self.controller.blend_phase, BlendPhase.LKAS)
    self.assertIn(BlendPhase.LTA_WINDDOWN, seen_phases)
    self.assertIn(BlendPhase.LTA_RELEASE, seen_phases)
    # the torque interface reached the demanded torque
    self.assertGreater(self.controller.last_torque, 800)

    # calm again: hands back to LTA after the dwell, ramping torque out first
    seen_phases = set()
    for _ in range(500):
      self._step(lat_active=True, torque=0.05, angle_deg=0.2)
      seen_phases.add(self.controller.blend_phase)
    self.assertEqual(self.controller.blend_phase, BlendPhase.LTA)
    self.assertIn(BlendPhase.LKAS_WINDDOWN, seen_phases)

    # disengage
    self.safety.set_controls_allowed(False)
    for _ in range(20):
      self._step(lat_active=False, torque=0.0)

  def test_no_simultaneous_actuation_on_bus(self):
    """Reconstruct the bus state frame by frame: the two interfaces must never actuate at once."""
    self.safety.set_controls_allowed(True)

    lka_active = False
    lta_active = False
    demand = [(0.1, 200), (0.7, 300), (0.05, 500), (0.65, 300)]
    for torque, frames in demand:
      for _ in range(frames):
        self.safety.set_timer(self.frame * 10000)
        self._rx_car_state()
        CS = self._cs_stub(True)
        CC = self._cc(True, torque, 0.5)
        _, can_sends = self.controller.update(CC, CS, self.frame * int(DT_CTRL * 1e9))

        for addr, dat, bus in can_sends:
          if addr in (0x2E4, 0x191):
            msg = libsafety_py.make_CANPacket(addr, bus, dat)
            self.assertTrue(self.safety.safety_tx_hook(msg))
            if addr == 0x2E4:
              lka_active = (dat[0] & 0x01) or (dat[1] != 0) or (dat[2] != 0)  # STEER_REQUEST | torque
            else:
              lta_active = bool(dat[0] & 0x01) or bool(dat[3] & 0x02)  # STEER_REQUEST | STEER_REQUEST_2
            self.assertFalse(lka_active and lta_active,
                             f"both interfaces actuating at frame {self.frame}")

        phase = self.controller.blend_phase
        if phase == BlendPhase.LTA:
          self.eps_torque = min(self.eps_torque + 15, 200)
        elif phase in (BlendPhase.LTA_WINDDOWN, BlendPhase.LTA_RELEASE):
          self.eps_torque = max(self.eps_torque - 15, 0)
        else:
          self.eps_torque = float(self.controller.last_torque)
        self.frame += 1


if __name__ == "__main__":
  unittest.main()
