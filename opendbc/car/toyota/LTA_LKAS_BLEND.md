# Blended LTA/LKAS steering for the TSS2 Corolla (prototype)

## Background

TSS2 Toyotas expose two lateral control interfaces on the powertrain bus:

| | LKAS (torque) | LTA (angle) |
|---|---|---|
| Message | `STEERING_LKA` (0x2E4), sent at 100 Hz | `STEERING_LTA` (0x191), sent at 50 Hz |
| Command | `STEER_TORQUE_CMD` (± 1500 units) | `STEER_ANGLE_CMD` (± 94.9°) |
| Character | direct torque authority, good for sustained curves and high lateral demand | EPS closes the angle loop internally, good for precise small corrections |
| Torque source | openpilot commands it explicitly | EPS applies whatever torque its internal (rate-limited, ~1500 units/s) controller needs, up to its own limits |
| Torque cut | `STEER_REQUEST=0` | `TORQUE_WIND_DOWN=0` ramps torque out at ~1500 units/s while still tracking |

Stock openpilot treats these as **mutually exclusive modes** selected at fingerprint
time (`ToyotaSafetyFlags.LTA`): torque cars must send a fully inactive 0x191, angle cars
must send a fully inactive 0x2E4. The TSS2 Corolla runs torque mode.

This prototype adds a third, opt-in mode — **blend** — for the TSS2 Corolla only:
minute corrections ride on the LTA angle interface; torquey maneuvers hand off to the
LKAS torque interface; and the whole thing stays inside the LKAS torque envelope.

## Architecture

The split is **arbitration, not superposition**: at any instant exactly one interface
actuates. Superposition (both actuating at once) was rejected because the EPS's summing
behavior is undocumented, and because it would make the "total torque ≤ LKAS limits"
argument unverifiable from the bus.

### Controller state machine (`carcontroller.py`)

```
            demand ≥ 600 units for 100 ms
   ┌─────┐  (or EPS torque ≥ 600)   ┌──────────────┐
   │ LTA │ ────────────────────────▶│ LTA_WINDDOWN │  TORQUE_WIND_DOWN=0,
   └─────┘                          └──────┬───────┘  EPS ramps torque out
      ▲                                    │ EPS torque ≤ 150 (or 0.5 s timeout)
      │ LKAS torque reached 0              ▼
┌───────────────┐                   ┌─────────────┐
│ LKAS_WINDDOWN │                   │ LTA_RELEASE │  one 0x191 frame with
└───────────────┘                   └──────┬──────┘  STEER_REQUEST=0
      ▲ demand ≤ 300 units for 2 s         │ next frame
      │ and ≥ 1 s in LKAS                  ▼
   ┌──┴───┐◀───────────────────────────────┘
   │ LKAS │   torque ramps up from 0 under stock rate limits
   └──────┘
```

- **Demand signal**: the unlimited torque request from the torque lateral controller
  (`actuators.torque × STEER_MAX`), i.e. the same tuning that drives LKAS today.
- **Hysteresis**: handoff up at 600 units sustained 100 ms; handback down at 300 units
  sustained 2 s plus a 1 s minimum dwell in LKAS. Handoff *to* LKAS is never delayed by
  a dwell — moving to the higher-authority interface is the safe direction.
- **Angle target**: `actuators.steeringAngleDeg`, produced for torque cars by a 2-line
  openpilot patch to `latcontrol_torque.py` (same VM curvature→angle conversion the
  angle controller uses). While LTA does not own actuation, the angle command tracks the
  measured angle, exactly as safety requires of an inactive angle command.
- **Torque continuity at handoffs**: the releasing interface always ramps to (near)
  zero before the acquiring interface ramps up from zero — LTA via `TORQUE_WIND_DOWN=0`
  (~1500 units/s, EPS-enforced), LKAS via the stock 25 units/frame down rate. There is a
  brief (~100–300 ms) low-assist gap by design; see Known unknowns.
- **Message ordering**: within a control frame openpilot emits 0x2E4 before 0x191, and
  panda evaluates them in order. LKAS→LTA can therefore hand off within one frame (the
  zero-torque 0x2E4 is seen first); LTA→LKAS needs the extra `LTA_RELEASE` frame so the
  `STEER_REQUEST=0` 0x191 is on the bus before torque starts. The state machine encodes
  both.

### Safety mode (`opendbc/safety/modes/toyota.h`)

New param `TOYOTA_PARAM_LTA_BLEND` (bit 4 of the flag byte, `16 << 8`), gated behind
`ALLOW_DEBUG` (debug firmware builds only) while this is a prototype. If set together
with `TOYOTA_PARAM_LTA` or `TOYOTA_PARAM_SECOC` it is ignored (fail-closed to the
stricter existing behavior).

In blend mode **no existing check is relaxed**; the mode is the conjunction of both
standalone rule sets plus two new rules:

1. **Full LKAS torque rule set** on 0x2E4 (`steer_torque_cmd_checks` with the stock
   `TOYOTA_TORQUE_STEERING_LIMITS`): ±1500 max, 15/frame up, 25/frame down, ±350 vs
   measured EPS torque, 450-unit real-time rate cap, steer-request-cut accounting.
2. **Full LTA angle rule set** on 0x191: ISO 11270-derived speed-dependent angle rate
   limits, 94.9° cap, request-bit consistency, wind-down gating, ≤150-unit driver
   torque, inactive command must match the measured angle. RX checks use the LTA
   variant (angle quality flag enforced).
3. **Mutual exclusion (new)**: an actuating 0x2E4 (`STEER_REQUEST` or torque ≠ 0) is
   blocked while the last bus-visible 0x191 had a steer request, and vice versa. The
   tracked state is updated **only from messages that pass all checks** (bus truth), so
   spam cannot flip it.
4. **Tighter LTA authority cap (new)**: `TORQUE_WIND_DOWN=100` is blocked when measured
   EPS torque exceeds **700 units** (vs 1500 in pure LTA mode), forcing a torque
   wind-down. The controller hands off at 600 to keep margin.

A message blocked by rule 3 is rejected **before** the underlying checker runs, so a
blocked message cannot advance rate-limit anchors (`desired_torque_last`,
`desired_angle_last`). Without this, spamming blocked commands during the other
interface's phase would ratchet the anchor and allow a step command after handoff;
`test_blend_no_*_state_ratchet_when_blocked` covers exactly that.

## Why this stays inside the LKAS torque limits

The user-facing safety claim is: **at no time does commanded assist exceed what the
stock LKAS safety mode already permits.**

- **LKAS phases**: literally the stock checks with the stock limits (rule 1). Nothing new.
- **LTA phases**: openpilot commands no torque (enforced zero, rule 3). The only torque
  is what the EPS itself applies to track the angle, which is:
  - observable on the bus (`STEER_TORQUE_SENSOR.STEER_TORQUE_EPS`, the same signal the
    stock torque checks measure against), and
  - capped by rule 4 at 700 units — less than half the 1500-unit LKAS envelope. Above
    that, panda only allows wind-down messages, and the EPS ramps torque out at
    ~1500 units/s. The angle trajectory itself is bounded by the ISO 11270-derived
    rate/accel limits of rule 2, which bound lateral jerk/accel regardless of torque.
- **Handoffs**: interfaces never actuate simultaneously (rule 3, verified per-message in
  the closed-loop test). During LTA→LKAS the residual EPS torque is ≤150 units at
  release (controller waits for wind-down; worst case ≤700 decaying at ~1500 units/s on
  the 0.5 s timeout path) while LKAS ramps up at ≤1500 units/s from zero under rule 1.
  The instantaneous sum is therefore always below the 1500-unit LKAS maximum: by the
  time the LKAS ramp could reach 1500 (≥1 s), any LTA residual has long decayed.
- **Driver override**: unchanged from the stricter of the two stock modes. LTA phases
  wind torque down above 150 units of driver torque (panda-enforced); LKAS phases have
  the stock measured-torque error bound; openpilot additionally drops lat control above
  500 units of driver torque.
- **Disengagement**: `controls_allowed=0` blocks actuation on both interfaces
  (stock rules), in every phase.

## Verification

All run from the opendbc repo root (`pytest` needs a working C compiler for the safety
model; see `opendbc/safety/tests/libsafety`):

- `opendbc/safety/tests/test_toyota.py::TestToyotaSafetyBlend` — the blend safety mode
  passes the **complete** stock torque test suite (`MotorTorqueSteeringSafetyTest`,
  `SteerRequestCutSafetyTest`) and the **complete** stock angle test suite
  (`AngleSteeringSafetyTest`, `test_lta_steer_cmd` with the tightened 700-unit
  threshold), plus:
  - `test_blend_mutual_exclusion` — actuation is exclusive; a handoff requires a
    zero-actuation message from the releasing interface first
  - `test_blend_no_torque_state_ratchet_when_blocked` / `test_blend_no_angle_state_ratchet_when_blocked`
  - `test_blend_flag_ignored_with_lta` — the param cannot weaken pure LTA mode
- `opendbc/car/toyota/tests/test_lta_lkas_blend.py` — closed-loop consistency: the real
  `CarController` is driven through engage → LTA → high demand → LKAS → calm → LTA →
  disengage with a simulated EPS, and **every** generated steering message is fed to the
  compiled safety model configured with `LTA_BLEND`. Nothing may be blocked, and a
  bus-state reconstruction asserts the two interfaces never actuate simultaneously.
- All 288 pre-existing Toyota tests still pass (no behavior change with the flag off).

## How to enable

1. TSS2 Corolla (`TOYOTA_COROLLA_TSS2`) only — other platforms are unaffected.
2. Set `TOYOTA_LTA_LKAS_BLEND=1` in the environment of the car interface process.
3. Panda must run a debug build (`ALLOW_DEBUG`), since the safety param is debug-gated.
4. Apply the 2-line openpilot patch to `selfdrive/controls/lib/latcontrol_torque.py`
   (returns the desired steering angle instead of `0.0`) so
   `actuators.steeringAngleDeg` is populated for torque cars.

## Known unknowns / road-test plan

These are exactly the things the prototype flags for on-car validation, in order:

1. **Angle tracking quality of the TSS2.0 Corolla EPS.** Stock Corolla LTA is a gentle
   lane-tracing function; whether the EPS tracks `STEER_ANGLE_CMD` crisply enough at a
   700-unit torque budget to be useful for "minute changes" is the core hypothesis.
   comma validated LTA control on TSS 2.5 (RAV4 2023); the Corolla is TSS 2.0.
   *Test: dashcam-mode CAN capture of stock LTA first, then low-speed lot testing.*
2. **`SETME_X3` semantics.** Stock TSS2.0 cameras send 3, TSS2.5 send 1; openpilot
   sends 1 when angle-controlling. The blend prototype sends 1 whenever the LTA
   interface is live. Confirm the Corolla EPS accepts angle actuation with it.
3. **EPS fault counters across handoffs.** Rapid alternation between interfaces is not
   something the stock system does. The dwell/hysteresis times are first guesses;
   verify no EPS diagnostic counters increment during repeated handoffs.
4. **Assist gap during LTA→LKAS handoff.** ~100–300 ms of reduced assist mid-maneuver.
   Measure lateral deviation on curve-entry handoffs; if objectionable, lower the
   handoff threshold (hand off earlier, while demand is still moderate) or add
   demand-rate prediction.
5. **Lateral tuning in the LTA regime.** The angle target currently comes through the
   torque controller's curvature pipeline with torque-car actuator delay (0.12 s vs the
   0.18 s used for angle cars). Expect to retune once tracking data exists.

## Files changed

- `opendbc/safety/modes/toyota.h` — blend safety mode
- `opendbc/safety/tests/test_toyota.py` — `TestToyotaSafetyBlend`
- `opendbc/car/toyota/carcontroller.py` — `BlendPhase` state machine + message wiring
- `opendbc/car/toyota/values.py` — flags, thresholds, `BLEND_STEER_CAR`
- `opendbc/car/toyota/interface.py` — opt-in wiring for the TSS2 Corolla
- `opendbc/car/toyota/tests/test_lta_lkas_blend.py` — closed-loop consistency test
- openpilot: `selfdrive/controls/lib/latcontrol_torque.py` — desired angle passthrough
