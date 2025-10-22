#!/usr/bin/env python3
"""
MINCO 轨迹可视化脚本（保存为图片）

读取 minco_trajectory_full.log 文件并可视化轨迹的各种属性，保存为 PNG 图片。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import sys
import os
import glob

def find_latest_trajectory_log():
    """查找最新的轨迹日志文件"""
    log_files = glob.glob('minco_trajectory_*.log')
    if not log_files:
        return None
    # 按修改时间排序，最新的在前
    log_files.sort(key=os.path.getmtime, reverse=True)
    return log_files[0]

def load_trajectory(filename=None):
    """
    加载轨迹数据和元数据

    格式: index, x, y, yaw, vx, vy, omega, acceleration, curvature, time, path_length
    """
    # 如果没有指定文件名，自动查找最新的
    if filename is None:
        filename = find_latest_trajectory_log()
        if filename is None:
            print(f"错误: 未找到任何轨迹日志文件 (minco_trajectory_*.log)")
            print(f"当前目录: {os.getcwd()}")
            print(f"请先运行 navsim_local_debug 生成轨迹日志文件")
            sys.exit(1)
        print(f"使用最新的轨迹日志文件: {filename}")

    if not os.path.exists(filename):
        print(f"错误: 文件 '{filename}' 不存在!")
        print(f"当前目录: {os.getcwd()}")
        print(f"请先运行 navsim_local_debug 生成轨迹日志文件")
        sys.exit(1)

    # 解析元数据
    metadata = {}
    data = []

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()

            # 跳过空行
            if not line:
                continue

            # 解析元数据（注释行）
            if line.startswith('#'):
                # 提取键值对 (例如: "# start_x: 0 m")
                if ':' in line:
                    parts = line[1:].split(':', 1)  # 去掉 '#' 并分割
                    if len(parts) == 2:
                        key = parts[0].strip()
                        value_str = parts[1].strip().split()[0]  # 取第一个词（去掉单位）
                        try:
                            metadata[key] = float(value_str)
                        except ValueError:
                            metadata[key] = value_str
                continue

            # 解析轨迹数据
            parts = [x.strip() for x in line.split(',')]
            if len(parts) >= 11:
                try:
                    data.append([float(x) for x in parts])
                except ValueError:
                    continue

    if not data:
        print(f"错误: 文件 '{filename}' 中没有有效数据!")
        sys.exit(1)

    data = np.array(data)

    trajectory = {
        'index': data[:, 0],
        'x': data[:, 1],
        'y': data[:, 2],
        'yaw': data[:, 3],
        'vx': data[:, 4],
        'vy': data[:, 5],
        'omega': data[:, 6],
        'acceleration': data[:, 7],
        'curvature': data[:, 8],
        'time': data[:, 9],
        'path_length': data[:, 10],
        'metadata': metadata,  # 添加元数据
    }

    print(f"✅ 成功加载 {len(data)} 个轨迹点")
    print(f"   时间范围: [{trajectory['time'][0]:.3f}, {trajectory['time'][-1]:.3f}] s")
    print(f"   路径长度: [{trajectory['path_length'][-1]:.3f}] m")
    print(f"   速度范围: [{np.min(trajectory['vx']):.3f}, {np.max(trajectory['vx']):.3f}] m/s")
    print(f"   元数据字段: {len(metadata)} 个")

    return trajectory

def visualize_trajectory(trajectory, output_file='minco_trajectory_visualization.png'):
    """
    可视化轨迹并保存为图片
    """
    # 创建图形窗口
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('MINCO Trajectory Visualization', fontsize=16, fontweight='bold')
    
    # 创建网格布局 (3 行 3 列)
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # ========== 1. XY 平面轨迹 (左上，占 2x2) ==========
    ax1 = fig.add_subplot(gs[0:2, 0:2])
    
    # 使用颜色映射显示速度
    scatter = ax1.scatter(trajectory['x'], trajectory['y'],
                         c=trajectory['vx'], cmap='viridis',
                         s=10, alpha=0.6)

    # 只标记终点（不标记起点，避免遮挡轨迹）
    ax1.plot(trajectory['x'][-1], trajectory['y'][-1], 'ro', markersize=12,
             label='Goal', markeredgecolor='black', markeredgewidth=2)
    
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title('XY Trajectory (colored by velocity)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend(loc='best')
    
    # 添加颜色条
    cbar = plt.colorbar(scatter, ax=ax1)
    cbar.set_label('Velocity (m/s)', fontsize=10)
    
    # ========== 2. 速度曲线 (右上) ==========
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(trajectory['time'], trajectory['vx'], 'b-', linewidth=1.5, label='vx')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # 绘制速度限制线
    meta = trajectory.get('metadata', {})
    max_vel = meta.get('max_vel', meta.get('max_velocity', None))
    min_vel = meta.get('min_vel', None)
    if max_vel is not None:
        ax2.axhline(y=max_vel, color='r', linestyle='--', linewidth=2, alpha=0.7, label=f'Max V: {max_vel:.2f} m/s')
    if min_vel is not None:
        ax2.axhline(y=min_vel, color='r', linestyle='--', linewidth=2, alpha=0.7, label=f'Min V: {min_vel:.2f} m/s')

    ax2.set_xlabel('Time (s)', fontsize=10)
    ax2.set_ylabel('Velocity (m/s)', fontsize=10)
    ax2.set_title('Linear Velocity vs Time', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=8)
    
    # ========== 3. 角速度曲线 (右中) ==========
    ax3 = fig.add_subplot(gs[1, 2])
    ax3.plot(trajectory['time'], trajectory['omega'], 'g-', linewidth=1.5, label='ω')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # 绘制角速度限制线
    max_omega = meta.get('max_omega', None)
    if max_omega is not None:
        ax3.axhline(y=max_omega, color='r', linestyle='--', linewidth=2, alpha=0.7, label=f'Max ω: {max_omega:.2f} rad/s')
        ax3.axhline(y=-max_omega, color='r', linestyle='--', linewidth=2, alpha=0.7, label=f'Min ω: {-max_omega:.2f} rad/s')

    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('Angular Velocity (rad/s)', fontsize=10)
    ax3.set_title('Angular Velocity vs Time', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='best', fontsize=8)
    
    # ========== 4. 加速度曲线 (左下) ==========
    ax4 = fig.add_subplot(gs[2, 0])
    ax4.plot(trajectory['time'], trajectory['acceleration'], 'r-', linewidth=1.5, label='acc')
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # 绘制加速度限制线
    max_acc = meta.get('max_acc', meta.get('max_acceleration', None))
    max_dec = meta.get('max_deceleration', None)
    if max_acc is not None:
        ax4.axhline(y=max_acc, color='r', linestyle='--', linewidth=2, alpha=0.7, label=f'Max A: {max_acc:.2f} m/s²')
    if max_dec is not None:
        ax4.axhline(y=-max_dec, color='r', linestyle='--', linewidth=2, alpha=0.7, label=f'Max D: {max_dec:.2f} m/s²')

    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Acceleration (m/s²)', fontsize=10)
    ax4.set_title('Acceleration vs Time', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='best', fontsize=8)
    
    # ========== 5. 曲率曲线 (中下) ==========
    ax5 = fig.add_subplot(gs[2, 1])
    
    # 过滤掉异常大的曲率值（速度接近0时的数值问题）
    curvature_filtered = trajectory['curvature'].copy()
    curvature_threshold = 100  # 曲率阈值
    curvature_filtered[np.abs(curvature_filtered) > curvature_threshold] = np.nan
    
    ax5.plot(trajectory['path_length'], curvature_filtered, 'm-', linewidth=1.5, label='κ')
    ax5.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax5.set_xlabel('Path Length (m)', fontsize=10)
    ax5.set_ylabel('Curvature (1/m)', fontsize=10)
    ax5.set_title('Curvature vs Path Length', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    ax5.legend(loc='best')
    
    # ========== 6. 朝向角曲线 (右下) ==========
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.plot(trajectory['time'], np.rad2deg(trajectory['yaw']), 'c-', linewidth=1.5, label='yaw')
    ax6.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax6.set_xlabel('Time (s)', fontsize=10)
    ax6.set_ylabel('Yaw (deg)', fontsize=10)
    ax6.set_title('Yaw Angle vs Time', fontsize=12, fontweight='bold')
    ax6.grid(True, alpha=0.3)
    ax6.legend(loc='best')
    
    # ========== 统计信息文本框 ==========
    meta = trajectory.get('metadata', {})

    # 起点和终点信息
    start_goal_text = f"""Start & Goal:
Start: ({meta.get('start_x', 0):.2f}, {meta.get('start_y', 0):.2f})
Goal: ({meta.get('goal_x', 0):.2f}, {meta.get('goal_y', 0):.2f})
Start V: {meta.get('start_vx', 0):.2f} m/s"""

    # 在 XY 图的左上角添加起点终点信息
    ax1.text(0.02, 0.98, start_goal_text.strip(), transform=ax1.transAxes,
             fontsize=8, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

    # 车辆参数信息
    vehicle_text = f"""Vehicle:
Model: {meta.get('chassis_model', 'N/A')}
Length: {meta.get('body_length', 0):.2f} m
Width: {meta.get('body_width', 0):.2f} m
Wheelbase: {meta.get('wheelbase', 0):.2f} m"""

    # 在 XY 图的右上角添加车辆信息
    ax1.text(0.98, 0.98, vehicle_text.strip(), transform=ax1.transAxes,
             fontsize=8, verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))

    # 约束信息
    constraints_text = f"""Constraints:
Max V: {meta.get('max_vel', 0):.2f} m/s
Max A: {meta.get('max_acc', 0):.2f} m/s²
Max ω: {meta.get('max_omega', 0):.2f} rad/s
Safe Dis: {meta.get('safe_distance', 0):.2f} m"""

    # 在 XY 图的左下角添加约束信息
    ax1.text(0.02, 0.02, constraints_text.strip(), transform=ax1.transAxes,
             fontsize=8, verticalalignment='bottom',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    # 轨迹统计信息
    stats_text = f"""Trajectory Stats:
Points: {len(trajectory['x'])}
Time: {trajectory['time'][-1]:.2f} s
Length: {trajectory['path_length'][-1]:.2f} m
V: [{np.min(trajectory['vx']):.2f}, {np.max(trajectory['vx']):.2f}] m/s
A: [{np.min(trajectory['acceleration']):.2f}, {np.max(trajectory['acceleration']):.2f}] m/s²"""

    # 在 XY 图的右下角添加统计信息
    ax1.text(0.98, 0.02, stats_text.strip(), transform=ax1.transAxes,
             fontsize=8, verticalalignment='bottom', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # 保存图片
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✅ 可视化图表已保存到: {output_file}")

def main():
    """
    主函数

    用法:
        python3 visualize_trajectory_save.py                          # 使用最新的日志文件
        python3 visualize_trajectory_save.py <log_file>               # 指定日志文件
        python3 visualize_trajectory_save.py <log_file> <output_png>  # 指定日志和输出文件
    """
    log_file = None
    output_file = 'minco_trajectory_visualization.png'

    # 如果提供了命令行参数，使用指定的文件
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    
    print("=" * 60)
    print("MINCO Trajectory Visualization (Save to File)")
    print("=" * 60)
    print(f"📂 加载轨迹文件: {log_file}")
    print(f"💾 输出图片文件: {output_file}")
    
    # 加载轨迹数据
    trajectory = load_trajectory(log_file)
    
    # 可视化轨迹
    print("\n📊 生成可视化图表...")
    visualize_trajectory(trajectory, output_file)
    
    print("\n✅ 完成!")

if __name__ == '__main__':
    main()

