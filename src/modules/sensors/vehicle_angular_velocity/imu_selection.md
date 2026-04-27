# IMU Selection Logic for VehicleAngularVelocity

`VehicleAngularVelocity` publishes `vehicle_angular_velocity`, which is the primary angular-rate
input to the rate controller. It reads raw gyroscope data from one of up to four hardware IMUs.
Selecting that IMU happens in two stages:

1. **System arbitration** — `VotedSensorsUpdate` (in the `sensors` module) evaluates all available
   IMUs every cycle and publishes the winner's device ID on the `sensor_selection` uORB topic.
2. **Local subscription** — `VehicleAngularVelocity::SensorSelectionUpdate()` reads that device ID,
   validates it, and subscribes to the matching `sensor_gyro_fifo` or `sensor_gyro` instance.

---

## Stage 1 — System-wide sensor arbitration

**Source file:** `src/modules/sensors/voted_sensors_update.cpp`

### Two operating modes (`SENS_IMU_MODE`)

`SENS_IMU_MODE` controls who drives the `sensor_selection` topic:

| Mode | Behaviour |
|------|-----------|
| **1** (default) | `VotedSensorsUpdate` runs its own confidence-and-priority voter and publishes the winner. |
| **0** | An external source (e.g. EKF2 via `estimator_selector_status`) writes `sensor_selection`. The voter still runs internally for inconsistency monitoring but does not change the selection. |

### Priority configuration

Each calibration slot has a configurable priority read from `CAL_GYROx_PRIO`. At startup and on
every parameter update, `VotedSensorsUpdate::parametersUpdate()` loops over all advertised
`vehicle_imu` instances, finds the matching calibration slot via
`calibration::FindCurrentCalibrationIndex("GYRO", device_id)`, and stores the priority.

- Setting a priority to **0** disables the sensor entirely — it is excluded from voting.
- On failover, the failed sensor's runtime priority is reduced to **1** (minimum enabled). This
  keeps it visible for inconsistency monitoring but prevents it from winning the vote unless it
  recovers to a high enough confidence to trigger rule 1 below.

### Confidence scoring (`DataValidator::confidence()`)

**Source file:** `src/modules/sensors/data_validator/DataValidator.cpp`

Each IMU instance produces a confidence value in \[0, 1\] every cycle:

| Condition | Flag | Confidence |
|-----------|------|------------|
| No data received yet | `ERROR_FLAG_NO_DATA` | 0.0 |
| Data older than timeout (40 ms normal, 500 ms HIL) | `ERROR_FLAG_TIMEOUT` | 0.0 |
| Same value repeated > 100 times | `ERROR_FLAG_STALE_DATA` | 0.0 |
| Cumulative error count > 10 000 | `ERROR_FLAG_HIGH_ERRCOUNT` | 0.0 |
| Error density exceeds window of 100 measurements | `ERROR_FLAG_HIGH_ERRDENSITY` | 0.0 |
| No errors | — | `1.0 − (error_density / 100)` |

### Best-sensor selection (`DataValidatorGroup::get_best()`)

**Source file:** `src/modules/sensors/data_validator/DataValidatorGroup.cpp`

Starting from the current best sensor, the voter scans all candidates and switches when **all**
of the following hold:

- Candidate confidence > 0
- One of these switch rules is satisfied:

| Rule | Condition | Notes |
|------|-----------|-------|
| 1 | Current confidence < 0.9 **and** candidate ≥ 0.9 | Switches back to a recovering sensor even if it has lower priority. |
| 2 | Candidate confidence > current **and** candidate priority ≥ current | Standard quality-driven promotion. |
| 3 | \|confidence difference\| < 1 % **and** candidate priority > current | Tie-break: prefer the operator-configured higher-priority sensor when quality is essentially equal. |

A switch driven by rule 3 or by a higher-priority sensor coming online is classified as a
**non-failsafe** transition. Rules 1/2 transitions driven by quality degradation are classified
as **failsafe** events: the failover counter increments and `checkFailover()` emits a MAVLink
emergency message.

### Inconsistency monitoring

`calcGyroInconsistency()` runs every cycle alongside the vote. For each enabled sensor it
computes an exponentially filtered difference from the fleet mean:

```
gyro_diff[i] = 0.95 × gyro_diff[i] + 0.05 × (gyro[i] − mean_of_all)
```

The vector magnitude is published as `sensors_status_imu.gyro_inconsistency_rad_s[i]`. This
metric does **not** directly drive sensor selection; it is surfaced for GCS display and EKF
health monitoring. The difference accumulators are zeroed whenever the primary changes so that
inconsistency is always measured relative to the current winner.

### `sensor_selection` publication

- Published only when `SENS_IMU_MODE=1` (or on the very first cycle before any external source
  has published, regardless of mode).
- Not published until both `accel_device_id` and `gyro_device_id` are non-zero, preventing
  downstream modules from acting on a partially-initialized selection.

---

## Stage 2 — Local subscription (`VehicleAngularVelocity::SensorSelectionUpdate()`)

**Source file:** `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp`

Triggered whenever `sensor_selection` updates, `_selected_sensor_device_id == 0`, or on a
forced refresh (startup or parameter change).

### Step 1 — Validate the upstream selection

Loops over `vehicle_imu_status[0..3]`. An instance is accepted as valid when:
- The topic is advertised with a non-zero timestamp.
- Data is no older than 1 s.
- `gyro_device_id` ≠ 0.

If the device ID from `sensor_selection` matches a valid instance, the selection proceeds. The
first valid instance found is saved as a last-resort fallback device ID. If the upstream
selection is absent or unhealthy, the fallback is used instead.

### Step 2 — Prefer FIFO subscription

Searches `sensor_gyro_fifo[0..3]` for an instance whose `device_id` matches the selected ID
(and whose data is fresh within 1 s). If found, registers a uORB callback on that instance
and sets `_fifo_available = true`.

FIFO is preferred because it carries the complete raw sample buffer at the hardware output data
rate (commonly 4–8 kHz). This gives the downstream low-pass and notch filters significantly
more samples per callback compared to the single-sample `sensor_gyro` path, improving filter
accuracy especially for dynamic notch tracking.

### Step 3 — Fall back to `sensor_gyro`

If no FIFO instance matches, searches `sensor_gyro[0..3]` for a device-ID match and registers
a callback on that instance. `_fifo_available` is set to `false`. The filter sample rate is
initialised to NaN and updated on the first data callback.

### Step 4 — Graceful failure

If neither subscription succeeds, `_selected_sensor_device_id` is set to 0 and a debug message
is logged. The rate controller continues to use the last valid `vehicle_angular_velocity` sample
until a sensor becomes available on the next update cycle.

---

## Parameters reference

### Selection and arbitration

| Parameter | Default | Range | Reboot? | Description |
|-----------|---------|-------|---------|-------------|
| `SENS_IMU_MODE` | 1 | 0–1 | Yes | 0 = follow external `sensor_selection` topic; 1 = internal voter publishes `sensor_selection` |
| `CAL_GYRO0_PRIO` | 50 | 0–100 | No | Voting priority for gyro calibration slot 0. 0 disables the sensor. Higher wins ties. |
| `CAL_GYRO1_PRIO` | 50 | 0–100 | No | Voting priority for gyro calibration slot 1 |
| `CAL_GYRO2_PRIO` | 50 | 0–100 | No | Voting priority for gyro calibration slot 2 |
| `CAL_GYRO3_PRIO` | 50 | 0–100 | No | Voting priority for gyro calibration slot 3 |
| `CAL_ACC0_PRIO` | 50 | 0–100 | No | Voting priority for accel calibration slot 0 (accel and gyro are selected as a matched IMU pair) |
| `CAL_ACC1_PRIO` | 50 | 0–100 | No | Voting priority for accel calibration slot 1 |
| `CAL_ACC2_PRIO` | 50 | 0–100 | No | Voting priority for accel calibration slot 2 |
| `CAL_ACC3_PRIO` | 50 | 0–100 | No | Voting priority for accel calibration slot 3 |

### Rate and filtering (applied after selection)

| Parameter | Default | Range | Unit | Reboot? | Description |
|-----------|---------|-------|------|---------|-------------|
| `IMU_GYRO_RATEMAX` | 400 | 100–2000 | Hz | Yes | Maximum publication rate for `vehicle_angular_velocity`. This is the rate controller loop rate. Raw sensor data is always read at the full hardware rate regardless of this value. |
| `IMU_GYRO_CUTOFF` | 40 | 0–1000 | Hz | No | 2nd-order Butterworth low-pass filter cutoff for the primary gyro. Applies to angular velocity sent to controllers only, not to estimators. 0 disables the filter. |
| `IMU_DGYRO_CUTOFF` | 20 | 0–1000 | Hz | No | Low-pass filter cutoff for the angular acceleration (D-term) signal. Filtering the derivative separately allows `IMU_GYRO_CUTOFF` to be raised, reducing control latency. |
| `IMU_GYRO_NF0_FRQ` | 0 | 0–1000 | Hz | No | Static notch filter 0 centre frequency. 0 disables the filter. |
| `IMU_GYRO_NF0_BW` | 20 | 0–100 | Hz | No | Static notch filter 0 bandwidth (−3 dB). Only active when `IMU_GYRO_NF0_FRQ` > 0. |
| `IMU_GYRO_NF1_FRQ` | 0 | 0–1000 | Hz | No | Static notch filter 1 centre frequency. 0 disables the filter. |
| `IMU_GYRO_NF1_BW` | 20 | 0–100 | Hz | No | Static notch filter 1 bandwidth (−3 dB). Only active when `IMU_GYRO_NF1_FRQ` > 0. |
| `IMU_GYRO_DNF_EN` | 0 | bitmask | — | No | Enable dynamic notch filters driven by ESC RPM telemetry (bit 0) or onboard FFT (bit 1). |
| `IMU_GYRO_DNF_BW` | 15 | 5–30 | Hz | No | Bandwidth of each dynamic notch filter. |
| `IMU_GYRO_DNF_HMC` | 3 | 1–7 | — | No | Number of harmonic notches tracked per ESC motor. |
| `IMU_GYRO_DNF_MIN` | 25 | — | Hz | No | Minimum frequency for dynamic notch filters (prevents them tracking near DC). |

---

## Key uORB topics

| Topic | Direction | Description |
|-------|-----------|-------------|
| `vehicle_imu[0..3]` | → `VotedSensorsUpdate` | Delta-angle and delta-velocity integrated samples from each `VehicleIMU` instance |
| `vehicle_imu_status[0..3]` | → both stages | Health status, error counts, and device IDs for each IMU instance |
| `sensor_selection` | ↔ between stages | Carries the winning `gyro_device_id` and `accel_device_id` |
| `sensors_status_imu` | ← `VotedSensorsUpdate` | Per-sensor inconsistency metrics, priorities, and health flags |
| `sensor_gyro_fifo[0..3]` | → `VehicleAngularVelocity` | Raw FIFO sample buffers from each gyro driver |
| `sensor_gyro[0..3]` | → `VehicleAngularVelocity` | Single-sample output from each gyro driver (non-FIFO fallback) |
| `vehicle_angular_velocity` | ← `VehicleAngularVelocity` | Calibrated, filtered angular velocity — primary rate controller input |

---

## Key source files

| File | Purpose |
|------|---------|
| `src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` | Stage 2: subscription management, calibration, and filtering |
| `src/modules/sensors/voted_sensors_update.cpp` | Stage 1: per-cycle voting, inconsistency monitoring, `sensor_selection` publication |
| `src/modules/sensors/data_validator/DataValidatorGroup.cpp` | `get_best()` — confidence-and-priority voting algorithm |
| `src/modules/sensors/data_validator/DataValidator.cpp` | Per-sensor confidence score and error-flag computation |
| `src/modules/sensors/vehicle_imu/VehicleIMU.cpp` | Produces `vehicle_imu` and `vehicle_imu_status` from raw driver topics |
