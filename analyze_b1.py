import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


TRUE_TOPIC = "/robot_gt"
EST_TOPIC  = "/robot_estimated_odometry"

# ---- metrics settings (tweak if needed)
CONV_THRESH_M = 0.20          # "converged" when position error < 20 cm
CONV_HOLD_SEC = 3.0           # must stay below threshold for 3 seconds
STABILITY_STD_WINDOW_SEC = 5.0  # compute std in last 5 seconds as "stability"
# ----------------------

def read_odom_xy_t(bag_path: str, topic: str):
    bag_path = str(Path(bag_path))
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = ConverterOptions(input_serialization_format="cdr",
                                         output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    if topic not in type_map:
        raise RuntimeError(f"Topic {topic} not found in {bag_path}. Available: {list(type_map.keys())}")

    msg_type = get_message(type_map[topic])

    ts = []
    xs = []
    ys = []

    while reader.has_next():
        (tname, data, tstamp_ns) = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(data, msg_type)
        # nav_msgs/Odometry: pose.pose.position.{x,y}
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ts.append(tstamp_ns * 1e-9)  # seconds
        xs.append(x)
        ys.append(y)

    return np.array(ts), np.array(xs), np.array(ys)

def align_by_nearest(t_true, x_true, y_true, t_est, x_est, y_est):
    # For each true timestamp, pick nearest estimate timestamp (simple + robust)
    idx = np.searchsorted(t_est, t_true, side="left")
    idx = np.clip(idx, 0, len(t_est)-1)

    # also check previous index to truly get nearest
    prev = np.clip(idx - 1, 0, len(t_est)-1)
    choose_prev = np.abs(t_est[prev] - t_true) < np.abs(t_est[idx] - t_true)
    idx = np.where(choose_prev, prev, idx)

    xe = x_est[idx]
    ye = y_est[idx]
    return t_true, x_true, y_true, xe, ye

def compute_metrics(t, x_true, y_true, x_est, y_est):
    err = np.sqrt((x_est - x_true)**2 + (y_est - y_true)**2)
    rmse = math.sqrt(float(np.mean(err**2)))

    # Convergence time: first time where error stays < thresh for HOLD seconds
    conv_time = None
    hold_samples = 1
    if len(t) >= 2:
        dt = np.median(np.diff(t))
        hold_samples = max(1, int(CONV_HOLD_SEC / max(dt, 1e-6)))

    below = err < CONV_THRESH_M
    for i in range(0, len(err) - hold_samples):
        if np.all(below[i:i+hold_samples]):
            conv_time = float(t[i] - t[0])  # seconds since start
            break

    # Stability: std dev of error in last window seconds (lower = more stable)
    t_end = t[-1]
    mask = t >= (t_end - STABILITY_STD_WINDOW_SEC)
    if np.any(mask):
        stability_std = float(np.std(err[mask]))
    else:
        stability_std = float(np.std(err))

    return rmse, conv_time, stability_std, err

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 analyze_b1.py <process_noise_value_label> <bag_path>")
        print("Example: python3 analyze_b1.py 1e-4 expB_proc_1e-4")
        sys.exit(1)

    label = sys.argv[1]
    bag = sys.argv[2]

    tT, xT, yT = read_odom_xy_t(bag, TRUE_TOPIC)
    tE, xE, yE = read_odom_xy_t(bag, EST_TOPIC)

    # normalize times to start at 0 for nicer plots
    t0 = min(tT[0], tE[0])
    tT -= t0
    tE -= t0

    t, x_true, y_true, x_est, y_est = align_by_nearest(tT, xT, yT, tE, xE, yE)
    rmse, conv_time, stability_std, err = compute_metrics(t, x_true, y_true, x_est, y_est)

    print(f"{label}\tRMSE(m)={rmse:.4f}\tConvergence(s)={conv_time if conv_time is not None else 'NO_CONV'}\tStabilityStd(m)={stability_std:.4f}")

    # Save an error-over-time plot per run
    plt.figure()
    plt.plot(t, err)
    plt.xlabel("time (s)")
    plt.ylabel("position error (m)")
    plt.title(f"Error over time (process noise={label})")
    plt.grid(True)
    out = f"err_{label}.png"
    plt.savefig(out, dpi=150)
    print(f"Saved {out}")

if __name__ == "__main__":
    main()
