"""Compare body vibration in saved robot traces; never connects to the robot.

Example:
    python tools/imu_shake.py --baseline tools/test_out/imu_stationary.csv \
        --input tools/test_out/steer_1_90.csv --output tools/test_out/imu_shake.json

The centered 0.3 s rolling mean removes slow intended motion. Remaining gyro
and acceleration RMS describe vibration at the IMU mounting point, not a safety
test or a mechanical pass/fail threshold. Repeated control samples are excluded
using each report's sequence number. Stale or absent streams have no RMS score.
"""

import argparse
import csv
import json
import math
from pathlib import Path


STREAMS = {
    "gyro": (1, "gyro_seq", ("gx_dps", "gy_dps", "gz_dps"), "deg/s"),
    "accel": (2, "accel_seq", ("ax_mps2", "ay_mps2", "az_mps2"), "m/s^2"),
}


def number(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else None
    except (KeyError, ValueError, TypeError):
        return None


def integer(row, key):
    value = number(row, key)
    return int(value) if value is not None and value.is_integer() else None


def read_trace(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if not reader.fieldnames or "t_ms" not in reader.fieldnames:
            raise ValueError(f"{path}: missing t_ms column")
        return list(reader)


def selected(row, mode):
    flags = integer(row, "flags")
    if flags is None:
        return False
    state = flags & 3
    if mode == "stationary":
        return state == 0 and number(row, "pwm") == 0
    return state == {"steer": 1, "drive": 2}[mode]


def trace_timing(rows):
    times = [number(row, "t_ms") for row in rows]
    gaps = [right - left for left, right in zip(times, times[1:])
            if left is not None and right is not None and right >= left]
    valid = [value for value in times if value is not None]
    return {
        "rows": len(rows),
        "duration_s": (valid[-1] - valid[0]) / 1000 if len(valid) > 1 else None,
        "max_trace_gap_ms": max(gaps) if gaps else None,
        "non_increasing_timestamps": sum(
            right <= left for left, right in zip(times, times[1:])
            if left is not None and right is not None),
    }


def highpass(groups, window_s):
    """Return centered-mean residuals, without joining separate motion periods."""
    residuals = []
    for group in groups:
        prefix = [[0.0, 0.0, 0.0]]
        for _, values in group:
            prefix.append([prefix[-1][axis] + values[axis] for axis in range(3)])
        left = right = 0
        for time_s, values in group:
            while left < len(group) and group[left][0] < time_s - window_s / 2:
                left += 1
            while right < len(group) and group[right][0] <= time_s + window_s / 2:
                right += 1
            # A short burst or a lone sample cannot support this filter window.
            if right - left < 3 or group[right - 1][0] - group[left][0] < window_s * 0.45:
                continue
            residuals.append(tuple(
                values[axis] - (prefix[right][axis] - prefix[left][axis]) / (right - left)
                for axis in range(3)))
    return residuals


def stream_metrics(rows, mode, stream, window_s):
    bit, seq_key, axes, unit = STREAMS[stream]
    required = ("imu_flags", seq_key, *axes)
    selected_rows = sum(selected(row, mode) for row in rows)
    result = {"available": False, "unit": unit, "selected_control_samples": selected_rows}
    missing = [key for key in required if not rows or key not in rows[0]]
    if missing:
        result["reason"] = "Missing columns: " + ", ".join(missing)
        return result
    groups = []
    group = None
    previous_seq = None
    previous_time = None
    fresh_count = 0
    duplicate_count = 0
    # A large gap starts a new filter segment. No interpolation crosses it.
    gap_limit_s = max(0.10, window_s / 2)
    for row in rows:
        seq = integer(row, seq_key)
        new_report = seq is not None and seq >= 0 and seq != previous_seq
        if seq is not None:
            previous_seq = seq
        time_ms = number(row, "t_ms")
        flags = integer(row, "imu_flags")
        values = tuple(number(row, axis) for axis in axes)
        valid = (selected(row, mode) and time_ms is not None and flags is not None
                 and bool(flags & bit) and all(value is not None for value in values)
                 and seq is not None and seq >= 0)
        if not valid:
            group = None
            previous_time = None
            continue
        fresh_count += 1
        if not new_report:
            duplicate_count += 1
            continue
        time_s = time_ms / 1000
        if group is None or (previous_time is not None and
                             (time_s <= previous_time or time_s - previous_time > gap_limit_s)):
            group = []
            groups.append(group)
        group.append((time_s, values))
        previous_time = time_s
    unique_count = sum(len(group) for group in groups)
    pairs = sum(max(0, len(group) - 1) for group in groups)
    span_s = sum(group[-1][0] - group[0][0] for group in groups if len(group) > 1)
    sample_gaps = [(right[0] - left[0]) * 1000 for group in groups
                   for left, right in zip(group, group[1:])]
    result.update({
        "fresh_control_samples": fresh_count,
        "fresh_coverage_fraction": fresh_count / selected_rows if selected_rows else None,
        "duplicate_control_samples_excluded": duplicate_count,
        "unique_observed_reports": unique_count,
        "observed_report_rate_hz": pairs / span_s if span_s > 0 else None,
        "max_observed_report_gap_ms": max(sample_gaps) if sample_gaps else None,
        "filter_segments": len(groups),
    })
    residuals = highpass(groups, window_s)
    if not residuals:
        result["reason"] = ("No fresh unique reports" if not unique_count else
                            "Too few continuous unique samples for the filter window")
        return result
    vector_squared = [sum(value * value for value in row) for row in residuals]
    result.update({
        "available": True,
        "analyzed_unique_reports": len(residuals),
        "highpass_vector_rms": math.sqrt(sum(vector_squared) / len(vector_squared)),
        "highpass_vector_peak": math.sqrt(max(vector_squared)),
        "highpass_axis_rms": [math.sqrt(sum(row[axis] ** 2 for row in residuals) / len(residuals))
                              for axis in range(3)],
    })
    return result


def analyze(rows, mode, window_s=0.3):
    return {"mode": mode, "timing": trace_timing(rows),
            **{stream: stream_metrics(rows, mode, stream, window_s) for stream in STREAMS}}


def compare_to_baseline(result, baseline):
    for stream in STREAMS:
        current, reference = result[stream], baseline[stream]
        if current["available"] and reference["available"]:
            baseline_rms = reference["highpass_vector_rms"]
            current["baseline_highpass_vector_rms"] = baseline_rms
            if baseline_rms > 1e-9:
                current["rms_ratio_to_baseline"] = current["highpass_vector_rms"] / baseline_rms


def self_test():
    def synthetic(oscillating=False):
        rows = []
        for index in range(400):
            # 100 Hz control traces with 50 Hz reports and an 8-bit sequence wrap.
            report = index // 2
            time_s = report * 0.02
            shake = math.sin(2 * math.pi * 5 * time_s) if oscillating else 0
            rows.append({"t_ms": index * 10, "flags": 0, "pwm": 0,
                         "imu_flags": 3, "gyro_seq": report % 256,
                         "accel_seq": report % 256,
                         "gx_dps": 3 + shake, "gy_dps": 2, "gz_dps": -4,
                         "ax_mps2": 0.1 + shake * 0.2, "ay_mps2": 0.2, "az_mps2": -0.1})
        return rows

    quiet = analyze(synthetic(), "stationary")
    shaking_rows = synthetic(True)
    shaking = analyze(shaking_rows, "stationary")
    assert quiet["gyro"]["highpass_vector_rms"] < 1e-10
    assert quiet["accel"]["highpass_vector_rms"] < 1e-10
    assert shaking["gyro"]["highpass_vector_rms"] > 0.6
    assert shaking["accel"]["highpass_vector_rms"] > 0.12
    assert shaking["gyro"]["unique_observed_reports"] == 200
    assert shaking["gyro"]["duplicate_control_samples_excluded"] == 200
    assert abs(shaking["gyro"]["observed_report_rate_hz"] - 50) < 1e-8
    for row in shaking_rows:
        row["imu_flags"] = 0
    stale = analyze(shaking_rows, "stationary")
    assert not stale["gyro"]["available"] and "highpass_vector_rms" not in stale["gyro"]
    # Force an actual sequence wrap within the sample and retain every unique report.
    wrapped_rows = synthetic(True)
    for index, row in enumerate(wrapped_rows):
        row["gyro_seq"] = (240 + index // 2) % 256
    assert analyze(wrapped_rows, "stationary")["gyro"]["unique_observed_reports"] == 200
    missing = analyze([{"t_ms": 0, "flags": 0, "pwm": 0}], "stationary")
    assert not missing["accel"]["available"]
    print("IMU analyzer self-test PASS: bias, 5 Hz shake, duplicates, stale data and sequence wrap")


def describe(label, result):
    parts = []
    for name in STREAMS:
        stream = result[name]
        if stream["available"]:
            rate = stream["observed_report_rate_hz"]
            rate_text = f"{rate:.1f} Hz" if rate is not None else "rate unknown"
            parts.append(f"{name} RMS {stream['highpass_vector_rms']:.4f} {stream['unit']}, "
                         f"peak {stream['highpass_vector_peak']:.4f}, {rate_text}, "
                         f"fresh {100 * stream['fresh_coverage_fraction']:.1f}%")
        else:
            parts.append(f"{name}: {stream['reason']}")
    print(f"{label} [{result['mode']}]: " + "; ".join(parts))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--baseline", help="Stationary trace with flags=0 and pwm=0")
    parser.add_argument("--input", nargs="+", action="append", help="Motion trace CSV files (may repeat)")
    parser.add_argument("--mode", choices=("auto", "steer", "drive"), default="auto",
                        help="Auto reports each active motion mode separately")
    parser.add_argument("--label", help="Optional label for this comparison")
    parser.add_argument("--window-s", type=float, default=0.3, help="Centered rolling-mean window, seconds")
    parser.add_argument("--output", help="Save comparison JSON")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        self_test()
        if not args.baseline and not args.input:
            return 0
    if not args.baseline or not args.input:
        parser.error("--baseline and --input are required, except for --self-test")
    if not math.isfinite(args.window_s) or args.window_s < 0.06:
        parser.error("--window-s must be finite and at least 0.06")
    try:
        baseline = analyze(read_trace(args.baseline), "stationary", args.window_s)
        baseline["path"] = str(Path(args.baseline).resolve())
        describe("baseline", baseline)
        comparisons = []
        for path in (path for group in args.input for path in group):
            rows = read_trace(path)
            modes = ([mode for mode in ("steer", "drive") if any(selected(row, mode) for row in rows)]
                     if args.mode == "auto" else [args.mode])
            if not modes:
                modes = ["steer"]  # Gives an explicit no-motion/no-data result.
            for mode in modes:
                result = analyze(rows, mode, args.window_s)
                result["path"] = str(Path(path).resolve())
                compare_to_baseline(result, baseline)
                comparisons.append(result)
                describe(Path(path).name, result)
        report = {
            "label": args.label,
            "method": {"filter": "centered rolling mean removed from each axis",
                       "window_s": args.window_s,
                       "rms": "sqrt(mean(gx^2 + gy^2 + gz^2)) of filter residuals, likewise acceleration",
                       "deduplication": "one sample per fresh observed sequence number; wraps are accepted",
                       "limits": "No pass/fail threshold. Only vibration at the IMU mount is measured. "
                                 "Bandwidth is below half the observed report rate. Slow motion is removed. "
                                 "Missing or stale data receives no numerical vibration score."},
            "baseline": baseline, "comparisons": comparisons,
        }
        if args.output:
            output = Path(args.output)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
            print(f"Saved {output.resolve()}")
    except (OSError, ValueError) as error:
        parser.exit(1, f"IMU analysis failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
