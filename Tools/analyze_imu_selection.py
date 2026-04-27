#!/usr/bin/env python3
"""
Analyze IMU selection events from a PX4 ULog file.

Prints a timeline of which gyro was selected, when selections changed, and
the most likely reason for each change based on sensor health, priority, and
error counts recorded in the log.

Usage:
    python3 analyze_imu_selection.py <log.ulg>
"""

import argparse
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    print("pyulog not found. Install it with:  pip install pyulog")
    sys.exit(1)

MAX_IMUS = 4

# Threshold below which inconsistency is flagged as notable (rad/s)
INCONSISTENCY_WARN_RAD_S = 0.1


def fmt_id(dev_id):
    if dev_id == 0:
        return "none"
    return f"0x{int(dev_id):08X}"


def nearest_idx(ts_array, target):
    """Return the index in ts_array whose value is closest to target."""
    idx = int(np.searchsorted(ts_array, target))
    if idx == 0:
        return 0
    if idx >= len(ts_array):
        return len(ts_array) - 1
    # Use Python int subtraction to avoid uint64 underflow
    if abs(int(ts_array[idx]) - int(target)) < abs(int(ts_array[idx - 1]) - int(target)):
        return idx
    return idx - 1


def load(ulog, topic, instance=0):
    """Return the data dict for a topic/instance, or None if absent."""
    try:
        return ulog.get_dataset(topic, instance).data
    except Exception:
        return None


def field(data, name, idx):
    """Safely read data[name][idx], returning None if missing."""
    if data is None or name not in data or idx >= len(data[name]):
        return None
    return data[name][idx]


# ---------------------------------------------------------------------------

def build_imu_slot_map(imu_statuses, status):
    """
    Return {gyro_device_id: slot_index} from vehicle_imu_status instances,
    falling back to sensors_status_imu array if no per-instance data exists.
    """
    slot_map = {}
    for slot, d in imu_statuses.items():
        if len(d["gyro_device_id"]) > 0:
            gid = int(d["gyro_device_id"][0])
            if gid != 0:
                slot_map[gid] = slot

    if not slot_map and status is not None:
        for s in range(MAX_IMUS):
            arr = status.get(f"gyro_device_ids[{s}]")
            if arr is not None and len(arr) > 0:
                gid = int(arr[0])
                if gid != 0:
                    slot_map[gid] = s

    return slot_map


def explain_change(prev_gyro_id, new_gyro_id, prev_slot, new_slot,
                   status, ts_us):
    """
    Return a list of human-readable reason strings describing why the
    selection changed, based on sensors_status_imu at ts_us.
    """
    reasons = []

    if status is None:
        return ["sensors_status_imu not logged — cannot determine reason"]

    si = nearest_idx(status["timestamp"], ts_us)

    def gyro_field(key, slot):
        return field(status, f"{key}[{slot}]", si)

    # Was the old sensor unhealthy at switch time?
    if prev_slot is not None:
        old_healthy = gyro_field("gyro_healthy", prev_slot)
        old_prio = gyro_field("gyro_priority", prev_slot)
        if old_healthy is not None and not bool(old_healthy):
            reasons.append(
                f"IMU slot {prev_slot} ({fmt_id(prev_gyro_id)}) became unhealthy"
                + (f" (priority reduced to {old_prio})" if old_prio is not None else "")
            )

    # Was the new sensor healthier?
    if new_slot is not None and prev_slot is not None:
        new_healthy = gyro_field("gyro_healthy", new_slot)
        new_prio = gyro_field("gyro_priority", new_slot)
        old_prio = gyro_field("gyro_priority", prev_slot)
        old_healthy = gyro_field("gyro_healthy", prev_slot)

        if new_healthy and old_healthy:
            # Both healthy — must be a priority/confidence tie-break
            if new_prio is not None and old_prio is not None:
                if new_prio > old_prio:
                    reasons.append(
                        f"IMU slot {new_slot} has higher priority "
                        f"({new_prio} > {old_prio}) with equal confidence"
                    )
                else:
                    reasons.append(
                        f"IMU slot {new_slot} ({fmt_id(new_gyro_id)}) had "
                        "higher confidence score than previous primary"
                    )
        elif new_healthy and old_healthy is not None and not old_healthy:
            reasons.append(
                f"IMU slot {new_slot} ({fmt_id(new_gyro_id)}) is healthy "
                "and was selected as replacement"
            )

    # Was inconsistency of the old sensor high?
    if prev_slot is not None:
        incons = gyro_field("gyro_inconsistency_rad_s", prev_slot)
        if incons is not None and incons > INCONSISTENCY_WARN_RAD_S:
            reasons.append(
                f"IMU slot {prev_slot} had notable inconsistency "
                f"({incons:.3f} rad/s > {INCONSISTENCY_WARN_RAD_S} rad/s threshold)"
            )

    if not reasons:
        reasons.append("insufficient logged data to determine exact reason")

    return reasons


def print_health_table(status, ts_us, highlight_id=None):
    """Print a per-slot health snapshot from sensors_status_imu near ts_us."""
    if status is None:
        return
    si = nearest_idx(status["timestamp"], ts_us)
    header = f"  {'Slot':<5} {'Gyro device':<14} {'Healthy':<9} {'Priority':<10} {'Inconsistency (rad/s)'}"
    print(header)
    print("  " + "-" * (len(header) - 2))
    for s in range(MAX_IMUS):
        gid_raw = field(status, f"gyro_device_ids[{s}]", si)
        if gid_raw is None or int(gid_raw) == 0:
            continue
        gid = int(gid_raw)
        healthy = field(status, f"gyro_healthy[{s}]", si)
        prio = field(status, f"gyro_priority[{s}]", si)
        incons = field(status, f"gyro_inconsistency_rad_s[{s}]", si)
        marker = " ◀ selected" if gid == highlight_id else ""
        healthy_str = ("yes" if bool(healthy) else "NO") if healthy is not None else "?"
        prio_str = str(int(prio)) if prio is not None else "?"
        incons_str = f"{incons:.4f}" if incons is not None else "?"
        print(f"  {s:<5} {fmt_id(gid):<14} {healthy_str:<9} {prio_str:<10} {incons_str}{marker}")


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Report which IMU was selected and why from a PX4 ULog"
    )
    parser.add_argument("logfile", help="Path to a .ulg log file")
    args = parser.parse_args()

    msg_filter = ["sensor_selection", "sensors_status_imu", "vehicle_imu_status"]

    print(f"Loading {args.logfile} …")
    try:
        ulog = ULog(args.logfile, msg_filter)
    except Exception as exc:
        print(f"Failed to open log: {exc}")
        sys.exit(1)

    sel = load(ulog, "sensor_selection")
    if sel is None:
        print("sensor_selection topic not found — was the sensors module running?")
        sys.exit(1)

    status = load(ulog, "sensors_status_imu")

    imu_statuses = {}
    for i in range(MAX_IMUS):
        d = load(ulog, "vehicle_imu_status", i)
        if d is not None:
            imu_statuses[i] = d

    slot_map = build_imu_slot_map(imu_statuses, status)  # gyro_id -> slot

    log_start_us = ulog.start_timestamp

    def rel_s(ts_us):
        # Cast to Python int to avoid numpy uint64 underflow
        diff = int(ts_us) - int(log_start_us)
        return diff * 1e-6

    # -----------------------------------------------------------------------
    # Header
    # -----------------------------------------------------------------------
    print()
    print("=" * 65)
    print("  IMU SELECTION ANALYSIS")
    print("=" * 65)

    # -----------------------------------------------------------------------
    # IMUs present
    # -----------------------------------------------------------------------
    print("\nIMUs found in log:")
    if slot_map:
        for gid, slot in sorted(slot_map.items(), key=lambda x: x[1]):
            d = imu_statuses.get(slot)
            aid = int(d["accel_device_id"][0]) if d is not None and len(d["accel_device_id"]) > 0 else 0
            gyro_rate = d["gyro_raw_rate_hz"][0] if d is not None and "gyro_raw_rate_hz" in d and len(d["gyro_raw_rate_hz"]) > 0 else float("nan")
            print(f"  Slot {slot}: gyro={fmt_id(gid)}  accel={fmt_id(aid)}"
                  + (f"  raw_rate={gyro_rate:.0f} Hz" if not np.isnan(gyro_rate) else ""))
    else:
        print("  (no vehicle_imu_status instances found)")

    # -----------------------------------------------------------------------
    # Selection timeline
    # -----------------------------------------------------------------------
    print()
    print("=" * 65)
    print("  SELECTION TIMELINE")
    print("=" * 65)

    sel_ts = sel["timestamp"]
    sel_gyro = sel["gyro_device_id"]
    sel_accel = sel["accel_device_id"]

    prev_gyro = None
    prev_accel = None
    change_count = 0

    for i in range(len(sel_ts)):
        gyro_id = int(sel_gyro[i])
        accel_id = int(sel_accel[i])

        if gyro_id == prev_gyro and accel_id == prev_accel:
            continue

        is_first = prev_gyro is None
        ts_s = rel_s(sel_ts[i])
        slot = slot_map.get(gyro_id)
        slot_str = f"slot {slot}" if slot is not None else "unknown slot"

        print()
        if is_first:
            print(f"[{ts_s:8.3f}s]  Initial selection")
        else:
            change_count += 1
            print(f"[{ts_s:8.3f}s]  *** Selection changed ***")

        print(f"  Gyro:  {fmt_id(gyro_id)}  ({slot_str})")
        print(f"  Accel: {fmt_id(accel_id)}")

        # Health table at this moment
        if status is not None:
            print()
            print_health_table(status, sel_ts[i], highlight_id=gyro_id)

        # Error counts for the deselected IMU
        if not is_first and prev_gyro is not None:
            prev_slot = slot_map.get(prev_gyro)
            if prev_slot is not None and prev_slot in imu_statuses:
                d = imu_statuses[prev_slot]
                si = nearest_idx(d["timestamp"], sel_ts[i])
                ec = field(d, "gyro_error_count", si)
                vib = field(d, "gyro_vibration_metric", si)
                temp = field(d, "temperature_gyro", si)
                parts = []
                if ec is not None:
                    parts.append(f"error_count={int(ec)}")
                if vib is not None:
                    parts.append(f"vibration={vib:.4f} rad/s")
                if temp is not None:
                    parts.append(f"temp={temp:.1f} °C")
                if parts:
                    print(f"\n  Deselected IMU (slot {prev_slot}) at switch time: {', '.join(parts)}")

        # Reasons
        if not is_first:
            prev_slot = slot_map.get(prev_gyro)
            reasons = explain_change(prev_gyro, gyro_id, prev_slot, slot,
                                     status, sel_ts[i])
            print()
            print("  Reason(s):")
            for r in reasons:
                print(f"    • {r}")

        prev_gyro = gyro_id
        prev_accel = accel_id

    # -----------------------------------------------------------------------
    # Final state
    # -----------------------------------------------------------------------
    print()
    print("=" * 65)
    print("  FINAL STATE")
    print("=" * 65)
    print()

    if prev_gyro is not None:
        final_slot = slot_map.get(prev_gyro)
        print(f"  Active gyro at end of log: {fmt_id(prev_gyro)}"
              + (f"  (slot {final_slot})" if final_slot is not None else ""))

    if status is not None:
        ts_end = status["timestamp"][-1]
        print("\n  Final health state:")
        print_health_table(status, ts_end, highlight_id=prev_gyro)

    print()
    print(f"  Total selection changes during flight: {change_count}")
    print()


if __name__ == "__main__":
    main()
