import pandas as pd

def analyze_log_simple(path, label=None):
    df = pd.read_csv(path)
    # 三大核心指标
    min_dist = df['closest_human_dist'].min()
    collision_count = df['collision_count'].iloc[-1]
    duration = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]

    name = label or path
    print(f"\n=== {name} ===")
    print(f"最小人机距离: {min_dist:.3f} m")
    print(f"碰撞次数: {collision_count}")
    print(f"任务完成时间: {duration:.2f} s")


def main():
     # 改成你的日志文件路径
    baseline_log = 'robot_log.csv'
    human_log = 'robot_log_True.csv'

    analyze_log_simple(baseline_log, label="Baseline")
    analyze_log_simple(human_log, label="Human-aware")

if __name__ == "__main__":
    main()