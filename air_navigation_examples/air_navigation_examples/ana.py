import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def load_log(path):
    df = pd.read_csv(path)

    # 确保关键列存在（兼容你两种版本）
    # 你新版本：有 collision_count
    # 旧版本：可能只有 human_collision 累积（不稳）
    for col in ["timestamp", "closest_human_dist", "collision_count"]:
        if col not in df.columns:
            df[col] = np.nan

    df["closest_human_dist"] = pd.to_numeric(df["closest_human_dist"], errors="coerce")
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    df["collision_count"] = pd.to_numeric(df["collision_count"], errors="coerce")

    df = df.replace([np.inf, -np.inf], np.nan)
    return df

def run_metrics(df):
    # min distance（过滤 NaN）
    min_dist = float(df["closest_human_dist"].min(skipna=True))

    # time：用 timestamp 的范围
    t0 = df["timestamp"].min(skipna=True)
    t1 = df["timestamp"].max(skipna=True)
    total_time = float(t1 - t0) if np.isfinite(t0) and np.isfinite(t1) else np.nan

    # collisions：用 collision_count 的最大值（累计次数）
    # 如果 collision_count 全空，再退化用 human_collision 的 sum(变化)（不推荐，但兜底）
    if df["collision_count"].notna().any():
        collisions = int(df["collision_count"].max(skipna=True))
    else:
        if "human_collision" in df.columns:
            hc = pd.to_numeric(df["human_collision"], errors="coerce").fillna(0).astype(int).values
            # 统计 0->1 的上升沿次数，避免连续帧重复计
            collisions = int(np.sum((hc[1:] == 1) & (hc[:-1] == 0)))
        else:
            collisions = 0

    return min_dist, total_time, collisions

def summarize_group(paths, group_name):
    mins, times, colls = [], [], []

    print(f"\n=== {group_name} runs ({len(paths)}) ===")
    for p in paths:
        df = load_log(p)
        m, t, c = run_metrics(df)
        mins.append(m); times.append(t); colls.append(c)
        print(f"- {os.path.basename(p):25s}  minDist={m:.3f}  time={t:.2f}s  collisions={c}")

    mins = np.array(mins, dtype=float)
    times = np.array(times, dtype=float)
    colls = np.array(colls, dtype=int)

    summary = {
        "group": group_name,
        "n_runs": len(paths),
        "mean_min_dist": float(np.nanmean(mins)),
        "std_min_dist": float(np.nanstd(mins, ddof=1)) if len(paths) > 1 else 0.0,
        "total_collisions": int(np.nansum(colls)),
        "mean_time": float(np.nanmean(times)),
        "std_time": float(np.nanstd(times, ddof=1)) if len(paths) > 1 else 0.0,
    }

    print(f"\n[{group_name} summary]")
    print(f"mean(min dist) = {summary['mean_min_dist']:.3f} ± {summary['std_min_dist']:.3f} m")
    print(f"total collisions = {summary['total_collisions']}")
    print(f"mean(time) = {summary['mean_time']:.2f} ± {summary['std_time']:.2f} s")

    return summary

def main():
    # 你把 run 文件放到 logs/ 里，然后命名：
    # logs/baseline_run1.csv, baseline_run2.csv ...
    # logs/aware_run1.csv, aware_run2.csv ...
    log_dir = os.path.expanduser("~/TDDE05/ros2_ws/logs")

    baseline_paths = sorted(glob.glob(os.path.join(log_dir, "baseline_run*.csv")))
    aware_paths    = sorted(glob.glob(os.path.join(log_dir, "aware_run*.csv")))

    # 如果你还没整理文件，也允许你直接用两个固定文件名
    if len(baseline_paths) == 0 and os.path.exists(os.path.expanduser("~/TDDE05/ros2_ws/robot_log.csv")):
        baseline_paths = [os.path.expanduser("~/TDDE05/ros2_ws/robot_log.csv")]

    if len(aware_paths) == 0 and os.path.exists(os.path.expanduser("~/TDDE05/ros2_ws/robot_log_with.csv")):
        aware_paths = [os.path.expanduser("~/TDDE05/ros2_ws/robot_log_with.csv")]

    if len(baseline_paths) == 0 or len(aware_paths) == 0:
        print("No logs found. Put files under ~/TDDE05/ros2_ws/logs/ as baseline_run*.csv and aware_run*.csv")
        print("Or set baseline/aware paths in the script.")
        return

    base_sum = summarize_group(baseline_paths, "Baseline")
    aware_sum = summarize_group(aware_paths, "Human-aware")

    # 保存一个汇总表，方便你粘到 report
    out = pd.DataFrame([base_sum, aware_sum])
    out_path = os.path.join(log_dir, "summary.csv")
    out.to_csv(out_path, index=False)
    print(f"\nSaved summary to: {out_path}")

    # 画图（可选）
    labels = ["Baseline", "Human-aware"]
    min_means = [base_sum["mean_min_dist"], aware_sum["mean_min_dist"]]
    time_means = [base_sum["mean_time"], aware_sum["mean_time"]]
    coll_totals = [base_sum["total_collisions"], aware_sum["total_collisions"]]

    fig1 = plt.figure()
    plt.bar(labels, min_means)
    plt.ylabel("mean(min distance) [m]")
    plt.title("Separation (multi-run mean)")
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, "mean_min_dist.png"), dpi=200)

    fig2 = plt.figure()
    plt.bar(labels, time_means)
    plt.ylabel("mean(time) [s]")
    plt.title("Task time (multi-run mean)")
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, "mean_time.png"), dpi=200)

    fig3 = plt.figure()
    plt.bar(labels, coll_totals)
    plt.ylabel("total collisions")
    plt.title("Collisions (sum over runs)")
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, "total_collisions.png"), dpi=200)

    print("Saved plots to logs/: mean_min_dist.png, mean_time.png, total_collisions.png")

if __name__ == "__main__":
    main()
