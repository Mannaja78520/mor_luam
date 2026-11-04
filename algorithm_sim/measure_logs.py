"""Cut the robot logs (tools/test_out/*.csv) into steer and drive episodes.

The measured values in detour_steer.RealismConfig come from this script:
    python measure_logs.py        (from algorithm_sim/, needs the repo's tools/test_out)
"""
import csv, glob, math, os, json

def unwrap_turn(a0, a1):
    return (a1 - a0) % 360.0

steers, drives = [], []
for f in sorted(glob.glob(r"E:\GPS_Localize\old\mor_luam\tools\test_out\*.csv")):
    try:
        rows = list(csv.DictReader(open(f)))
    except Exception:
        continue
    if not rows or "pwm" not in rows[0]:
        continue
    num = lambda r, k: float(r[k]) if r.get(k, "") not in ("", None) else 0.0
    t = [num(r, "t_ms") / 1000 for r in rows]
    pwm = [num(r, "pwm") for r in rows]
    ang = [num(r, "steer_deg") for r in rows]
    tgt = [num(r, "target_deg") for r in rows]
    x = [num(r, "x_m") for r in rows]; y = [num(r, "y_m") for r in rows]
    yaw = [num(r, "imu_yaw_deg") for r in rows] if "imu_yaw_deg" in rows[0] else None
    i, n = 0, len(rows)
    while i < n:
        if pwm[i] > 0:                                   # steering episode
            j = i
            while j < n and pwm[j] > 0:
                j += 1
            k = min(n - 1, j + 30)                       # 0.3 s after the motor stops: coast done
            turned = 0.0
            for m in range(i + 1, k + 1):
                d = (ang[m] - ang[m - 1] + 180) % 360 - 180
                turned += d
            err = (tgt[k] - ang[k] + 180) % 360 - 180    # + = still short, - = past
            steers.append(dict(file=os.path.basename(f), t0=t[i], dur=t[j - 1] - t[i], turned=turned,
                               err=err, start_err=(tgt[i] - ang[i]) % 360))
            i = j
        elif pwm[i] < 0:                                 # driving episode
            j = i
            while j < n and pwm[j] < 0:
                j += 1
            dist = math.hypot(x[j - 1] - x[i], y[j - 1] - y[i])
            wheel = sum((ang[m] - ang[m - 1] + 180) % 360 - 180 for m in range(i + 1, j))
            body = None
            if yaw:
                body = (yaw[j - 1] - yaw[i] + 180) % 360 - 180
            # lateral deviation from the straight line start -> end
            dx, dy = x[j - 1] - x[i], y[j - 1] - y[i]
            L = math.hypot(dx, dy) or 1e-9
            lat = max(abs(((x[m] - x[i]) * dy - (y[m] - y[i]) * dx) / L) for m in range(i, j))
            drives.append(dict(file=os.path.basename(f), dur=t[j - 1] - t[i], dist=dist,
                               wheel_change=wheel, body_change=body, max_lateral=lat))
            i = j
        else:
            i += 1

print(len(steers), "steers,", len(drives), "drives")
print("\nSTEERS (turned deg, time s, end error deg: + short / - past)")
for s in steers:
    if s["turned"] > 3:
        print(f"  {s['file']:<34} turned {s['turned']:6.1f}  time {s['dur']:5.2f}  end err {s['err']:+5.1f}")
print("\nDRIVES (m, s, wheel angle change, body yaw change, max lateral m)")
for d in drives:
    if d["dist"] > 0.02:
        b = "" if d["body_change"] is None else f"{d['body_change']:+5.1f}"
        print(f"  {d['file']:<34} {d['dist']:5.3f} m {d['dur']:5.1f} s  wheel {d['wheel_change']:+5.1f}  body {b:>6}  lat {d['max_lateral']*1000:4.1f} mm")
