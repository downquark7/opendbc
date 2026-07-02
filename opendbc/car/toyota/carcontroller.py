import math
import numpy as np
from opendbc.car import Bus, make_tester_present_msg, rate_limit, structs, ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.lateral import apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance
from opendbc.car.carlog import carlog
from opendbc.car.common.filter_simple import FirstOrderFilter, HighPassFilter
from opendbc.car.common.pid import PIDController
from opendbc.car.secoc import add_mac, build_sync_mac
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CAR, TSS2_CAR, UNSUPPORTED_DSU_CAR, CarControllerParams, ToyotaFlags
from opendbc.can import CANPacker

Ecu = structs.CarParams.Ecu
LongCtrlState = structs.CarControl.Actuators.LongControlState
SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# The up limit allows the brakes/gas to unwind quickly leaving a stop,
# the down limit roughly matches the rate of ACCEL_NET, reducing PCM compensation windup
ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -4.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_PID_UNWIND = 0.03 * DT_CTRL * 3  # m/s^2 / frame

MAX_PITCH_COMPENSATION = 1.5  # m/s^2

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 17  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500


class BlendPhase:
  """Phases of the blended LTA/LKAS arbitration state machine.

  Panda safety enforces that only one steering interface actuates at a time, and that a
  handoff is preceded by a zero-actuation message from the releasing interface. The
  wind-down/release phases below sequence the handoffs to respect that, and to avoid
  torque discontinuities: the releasing interface always ramps its torque to (near) zero
  before the other interface starts ramping up from zero.
  """
  LKAS = 0           # torque interface actuates, LTA request off, angle cmd tracks measured angle
  LTA = 1            # angle interface actuates, LKAS torque zero
  LTA_WINDDOWN = 2   # LTA still requested with TORQUE_WIND_DOWN=0: EPS ramps its torque out
  LTA_RELEASE = 3    # one LTA frame with STEER_REQUEST=0 before LKAS may actuate
  LKAS_WINDDOWN = 4  # LKAS torque ramps to zero before handing back to LTA


def get_long_tune(CP, params):
  if CP.carFingerprint in TSS2_CAR:
    kiBP = [2., 5.]
    kiV = [0.5, 0.25]
  else:
    kiBP = [0., 5., 35.]
    kiV = [3.6, 2.4, 1.5]

  return PIDController(0.0, (kiBP, kiV), k_f=1.0,
                       pos_limit=params.ACCEL_MAX, neg_limit=params.ACCEL_MIN,
                       rate=1 / (DT_CTRL * 3))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.last_torque = 0
    self.last_angle = 0
    self.alert_active = False
    self.standstill_req = False
    self.permit_braking = True
    self.steer_rate_counter = 0
    self.distance_button = 0

    # *** blended LTA/LKAS state (prototype) ***
    self.blend_enabled = bool(self.CP.flags & ToyotaFlags.LTA_LKAS_BLEND.value)
    self.blend_phase = BlendPhase.LTA
    self.blend_demand_frames = 0
    self.blend_calm_frames = 0
    self.blend_phase_frames = 0
    self.blend_winddown_frames = 0
    self.blend_release_sent = False

    # *** start long control state ***
    self.long_pid = get_long_tune(self.CP, self.params)
    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL * 3)
    self.pitch = FirstOrderFilter(0, 0.5, DT_CTRL)
    self.pitch_hp = HighPassFilter(0.0, 0.25, 1.5, DT_CTRL)

    self.accel = 0
    self.prev_accel = 0
    # *** end long control state ***

    self.packer = CANPacker(dbc_names[Bus.pt])

    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_acc_message_counter = 0
    self.secoc_prev_reset_counter = 0

  def update_blend_phase(self, lat_active, demand_torque, CS):
    """Advances the LTA/LKAS arbitration state machine, called once per 100Hz frame.

    demand_torque is the unlimited torque request from the lateral controller, in units
    of STEER_MAX. Small sustained demand runs on the LTA (angle) interface for precision;
    sustained demand above the handoff threshold, or EPS-applied torque near the blend-mode
    safety bound, hands off to the LKAS (torque) interface for authority.
    """
    p = self.params

    if not lat_active:
      # start the next engagement in the precision regime
      self.blend_phase = BlendPhase.LTA
      self.blend_demand_frames = 0
      self.blend_calm_frames = 0
      self.blend_phase_frames = 0
      self.blend_winddown_frames = 0
      self.blend_release_sent = False
      return

    self.blend_phase_frames += 1
    eps_torque = abs(CS.out.steeringTorqueEps)

    # demand persistence counters with hysteresis between the two thresholds
    if abs(demand_torque) >= p.BLEND_TO_LKAS_TORQUE or eps_torque >= p.BLEND_LTA_MAX_EPS_TORQUE:
      self.blend_demand_frames += 1
    else:
      self.blend_demand_frames = 0

    if abs(demand_torque) <= p.BLEND_TO_LTA_TORQUE and eps_torque < p.BLEND_LTA_MAX_EPS_TORQUE:
      self.blend_calm_frames += 1
    else:
      self.blend_calm_frames = 0

    if self.blend_phase == BlendPhase.LTA:
      # handing off to the higher-authority interface is never delayed by a dwell time
      if self.blend_demand_frames >= p.BLEND_DEMAND_FRAMES:
        self.blend_phase = BlendPhase.LTA_WINDDOWN
        self.blend_winddown_frames = 0

    elif self.blend_phase == BlendPhase.LTA_WINDDOWN:
      # TORQUE_WIND_DOWN=0 ramps EPS torque out at ~1500 units/s; wait for it before releasing
      self.blend_winddown_frames += 1
      if eps_torque <= p.BLEND_WINDDOWN_EPS_TORQUE or self.blend_winddown_frames >= p.BLEND_WINDDOWN_TIMEOUT:
        self.blend_phase = BlendPhase.LTA_RELEASE
        self.blend_release_sent = False

    elif self.blend_phase == BlendPhase.LTA_RELEASE:
      # LKAS torque may only start one frame after a STEER_REQUEST=0 LTA message went out,
      # since panda safety processes the LKAS message before the LTA message within a frame
      if self.blend_release_sent:
        self.blend_phase = BlendPhase.LKAS
        self.blend_phase_frames = 0

    elif self.blend_phase == BlendPhase.LKAS:
      if self.blend_calm_frames >= p.BLEND_CALM_FRAMES and self.blend_phase_frames >= p.BLEND_MIN_LKAS_FRAMES:
        self.blend_phase = BlendPhase.LKAS_WINDDOWN

    elif self.blend_phase == BlendPhase.LKAS_WINDDOWN:
      # abort the handback if demand returns while ramping down
      if self.blend_calm_frames == 0:
        self.blend_phase = BlendPhase.LKAS
      elif self.last_torque == 0:
        # torque interface fully released; LTA request turns on this frame. In-frame
        # message order (LKAS before LTA) makes panda see the release first
        self.blend_phase = BlendPhase.LTA
        self.blend_phase_frames = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])
      self.pitch_hp.update(CC.orientationNED[1])

    # *** control msgs ***
    can_sends = []

    # *** handle secoc reset counter increase ***
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_acc_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

        expected_mac = build_sync_mac(self.secoc_key, int(CS.secoc_synchronization['TRIP_CNT']), int(CS.secoc_synchronization['RESET_CNT']))
        if int(CS.secoc_synchronization['AUTHENTICATOR']) != expected_mac:
          carlog.error("SecOC synchronization MAC mismatch, wrong key?")

    # *** steer torque ***
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_meas_steer_torque_limits(new_torque, self.last_torque, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_torque = 0

    # *** blended LTA/LKAS arbitration ***
    lta_owns_actuation = False
    if self.blend_enabled:
      self.update_blend_phase(lat_active, new_torque, CS)
      lta_owns_actuation = self.blend_phase in (BlendPhase.LTA, BlendPhase.LTA_WINDDOWN)

      if self.blend_phase == BlendPhase.LKAS_WINDDOWN:
        # ramp the torque interface to zero before handing back to LTA
        apply_torque = apply_meas_steer_torque_limits(0, self.last_torque, CS.out.steeringTorqueEps, self.params)
      elif self.blend_phase != BlendPhase.LKAS:
        # the torque interface must be fully silent while the angle interface owns actuation
        apply_torque = 0
        apply_steer_req = False

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle or self.blend_enabled:
      if self.CP.steerControlType == SteerControlType.angle:
        # If using LTA control, disable LKA and set steering angle command
        apply_torque = 0
        apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # in blend mode the angle command only tracks the desired angle while LTA owns
        # actuation; otherwise it tracks the measured angle, as safety requires when inactive
        angle_control_active = CC.latActive if not self.blend_enabled else (CC.latActive and lta_owns_actuation)

        # Angular rate limit based on speed
        self.last_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw,
                                                       CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg,
                                                       angle_control_active, self.params.ANGLE_LIMITS)

    self.last_torque = apply_torque

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    steer_command = toyotacan.create_steer_command(self.packer, apply_torque, apply_steer_req)
    if self.CP.flags & ToyotaFlags.SECOC.value:
      # TODO: check if this slow and needs to be done by the CANPacker
      steer_command = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lka_message_counter,
                              steer_command)
      self.secoc_lka_message_counter += 1
    can_sends.append(steer_command)

    # STEERING_LTA does not seem to allow more rate by sending faster, and may wind up easier
    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      if self.blend_enabled:
        lta_active = lat_active and lta_owns_actuation
        # blend-mode safety blocks TORQUE_WIND_DOWN=100 above 700 units of EPS torque; keep margin
        max_lta_eps_torque = self.params.BLEND_LTA_MAX_EPS_TORQUE
        # the EPS acts on the LTA message per its SETME_X3 control-type semantics
        lta_control_type = SteerControlType.angle
      else:
        lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
        max_lta_eps_torque = self.params.STEER_MAX
        lta_control_type = self.CP.steerControlType
      # cut steering torque with TORQUE_WIND_DOWN when either EPS torque or driver torque is above
      # the threshold, to limit max lateral acceleration and for driver torque blending respectively.
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < max_lta_eps_torque and
                               abs(CS.out.steeringTorque) < self.params.MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      if self.blend_enabled and self.blend_phase == BlendPhase.LTA_WINDDOWN:
        # ramp EPS torque out ahead of the handoff to LKAS
        torque_wind_down = 0
      if self.blend_enabled and self.blend_phase == BlendPhase.LTA_RELEASE:
        # this frame's message carries STEER_REQUEST=0; LKAS may actuate starting next frame
        self.blend_release_sent = True

      can_sends.append(toyotacan.create_lta_steer_command(self.packer, lta_control_type, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

      if self.CP.flags & ToyotaFlags.SECOC.value:
        lta_steer_2 = toyotacan.create_lta_steer_command_2(self.packer, self.frame // 2)
        lta_steer_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lta_message_counter,
                              lta_steer_2)
        self.secoc_lta_message_counter += 1
        can_sends.append(lta_steer_2)

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

    # *** gas and brake ***
    if self.CP.openpilotLongitudinalControl:
      # if user engages at a stop with foot on brake, PCM starts in a special cruise standstill mode. on resume press,
      # brakes can take a while to ramp up causing a lurch forward. prevent resume press until planner wants to move.
      # don't use CC.cruiseControl.resume since it is gated on CS.cruiseState.standstill which goes false for 3s after resume press
      # whitelist hybrids as they do not have this issue and can stay stopped after resume press
      if not self.CP.flags & ToyotaFlags.HYBRID.value:
        should_resume = actuators.accel > 0
        if should_resume:
          self.standstill_req = False

        if not should_resume and CS.out.cruiseState.standstill:
          self.standstill_req = True

      if self.frame % 3 == 0:
        # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
        if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
          desired_distance = 4 - hud_control.leadDistanceBars
          if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
            self.distance_button = not self.distance_button
          else:
            self.distance_button = 0

        # internal PCM gas command can get stuck unwinding from negative accel so we apply a generous rate limit
        pcm_accel_cmd = actuators.accel
        if CC.longActive:
          pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
        self.prev_accel = pcm_accel_cmd

        # calculate amount of acceleration PCM should apply to reach target, given pitch.
        # clipped to only include downhill angles, avoids erroneously unsetting PERMIT_BRAKING when stopping on uphills
        accel_due_to_pitch = math.sin(min(self.pitch.x, 0.0)) * ACCELERATION_DUE_TO_GRAVITY
        # TODO: on uphills this sometimes sets PERMIT_BRAKING low not considering the creep force
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch

        # GVC does not overshoot ego acceleration when starting from stop, but still has a similar delay
        if not self.CP.flags & ToyotaFlags.SECOC.value:
          a_ego_blended = float(np.interp(CS.out.vEgo, [1.0, 2.0], [CS.gvc, CS.out.aEgo]))
        else:
          a_ego_blended = CS.out.aEgo

        # wind down integral when approaching target for step changes and smooth ramps to reduce overshoot
        prev_aego = self.aego.x
        self.aego.update(a_ego_blended)
        j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 3)

        future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.25, 0.5]))
        a_ego_future = a_ego_blended + j_ego * future_t

        if CC.longActive:
          # constantly slowly unwind integral to recover from large temporary errors
          self.long_pid.i -= ACCEL_PID_UNWIND * float(np.sign(self.long_pid.i))

          error_future = pcm_accel_cmd - a_ego_future

          if not stopping:
            # Toyota's PCM slowly responds to changes in pitch. On change, we amplify our
            # acceleration request to compensate for the undershoot and following overshoot
            pitch_compensation = float(np.clip(math.sin(self.pitch_hp.x) * ACCELERATION_DUE_TO_GRAVITY,
                                               -MAX_PITCH_COMPENSATION, MAX_PITCH_COMPENSATION))
            pcm_accel_cmd += pitch_compensation

          pcm_accel_cmd = self.long_pid.update(error_future,
                                               speed=CS.out.vEgo,
                                               feedforward=pcm_accel_cmd,
                                               freeze_integrator=actuators.longControlState != LongCtrlState.pid)
        else:
          self.long_pid.reset()

        # Along with rate limiting positive jerk above, this greatly improves gas response time
        # Consider the net acceleration request that the PCM should be applying (pitch included)
        net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        if net_acceleration_request_min < 0.2 or stopping or not CC.longActive:
          self.permit_braking = True
        elif net_acceleration_request_min > 0.3:
          self.permit_braking = False

        pcm_accel_cmd = float(np.clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX))

        main_accel_cmd = 0. if self.CP.flags & ToyotaFlags.SECOC.value else pcm_accel_cmd
        can_sends.append(toyotacan.create_accel_command(self.packer, main_accel_cmd, pcm_cancel_cmd, self.permit_braking, self.standstill_req, lead,
                                                        CS.acc_type, fcw_alert, self.distance_button))
        if self.CP.flags & ToyotaFlags.SECOC.value:
          acc_cmd_2 = toyotacan.create_accel_command_2(self.packer, pcm_accel_cmd)
          acc_cmd_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_acc_message_counter,
                              acc_cmd_2)
          self.secoc_acc_message_counter += 1
          can_sends.append(acc_cmd_2)

        self.accel = pcm_accel_cmd

    else:
      # we can spam can to cancel the system even if we are using lat only control
      if pcm_cancel_cmd:
        if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
          can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
        else:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead, CS.acc_type, False, self.distance_button))

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
