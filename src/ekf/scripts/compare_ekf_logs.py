#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
比較EKF估計輸出與PX4 ulog數據
用法：
    python3 compare_ekf_logs.py --ekf-log /path/to/ekf_log.csv --ulog /path/to/flight_log.ulg
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pyulog import ULog
from pyulog.px4 import message_definitions
import os
import sys
from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D

def load_ekf_log(ekf_log_path):
    """
    加載EKF CSV日誌文件
    返回一個包含EKF數據的Pandas DataFrame
    """
    print(f"加載EKF日誌: {ekf_log_path}")
    
    if not os.path.exists(ekf_log_path):
        print(f"錯誤: EKF日誌文件不存在: {ekf_log_path}")
        sys.exit(1)
    
    try:
        df = pd.read_csv(ekf_log_path)
        print(f"已加載 {len(df)} 條EKF數據記錄")
        return df
    except Exception as e:
        print(f"加載EKF日誌時出錯: {str(e)}")
        sys.exit(1)

def load_ulog_data(ulog_path):
    """
    加載PX4 ulog文件
    返回一個包含ulog數據的字典
    """
    print(f"加載PX4 ulog文件: {ulog_path}")
    
    if not os.path.exists(ulog_path):
        print(f"錯誤: ulog文件不存在: {ulog_path}")
        sys.exit(1)
    
    try:
        # 加載ulog文件
        ulog = ULog(ulog_path)
        
        # 提取感興趣的topics
        position_data = None
        attitude_data = None
        velocity_data = None
        
        # 提取本地位置數據
        for d in ulog.data_list:
            if d.name == 'vehicle_local_position':
                position_data = d.data
            elif d.name == 'vehicle_attitude':
                attitude_data = d.data
            elif d.name == 'vehicle_local_position_groundtruth':
                position_gt_data = d.data
            elif d.name == 'vehicle_attitude_groundtruth':
                attitude_gt_data = d.data
        
        if position_data is None:
            print("警告: ulog中未找到位置數據 (vehicle_local_position)")
        
        if attitude_data is None:
            print("警告: ulog中未找到姿態數據 (vehicle_attitude)")
        
        # 返回包含數據的字典
        return {
            'position': position_data,
            'attitude': attitude_data,
            'position_gt': position_gt_data if 'position_gt_data' in locals() else None,
            'attitude_gt': attitude_gt_data if 'attitude_gt_data' in locals() else None
        }
    
    except Exception as e:
        print(f"加載ulog文件時出錯: {str(e)}")
        sys.exit(1)

def synchronize_data(ekf_df, ulog_data):
    """
    同步EKF數據和ulog數據，使它們具有相同的時間戳
    返回同步後的數據框
    """
    print("同步EKF和ulog數據...")
    
    # 確保ulog數據存在
    if 'position' not in ulog_data or ulog_data['position'] is None:
        print("錯誤: ulog中沒有位置數據")
        return None
    
    # 從ulog中提取位置數據
    ulog_pos_df = pd.DataFrame({
        'timestamp': ulog_data['position']['timestamp'] / 1e6,  # 轉換為秒
        'ulog_pos_x': ulog_data['position']['x'],
        'ulog_pos_y': ulog_data['position']['y'],
        'ulog_pos_z': ulog_data['position']['z'],
        'ulog_vel_x': ulog_data['position']['vx'],
        'ulog_vel_y': ulog_data['position']['vy'],
        'ulog_vel_z': ulog_data['position']['vz']
    })
    
    # 如果存在姿態數據，也提取它
    if 'attitude' in ulog_data and ulog_data['attitude'] is not None:
        ulog_att_df = pd.DataFrame({
            'timestamp': ulog_data['attitude']['timestamp'] / 1e6,  # 轉換為秒
            'ulog_q0': ulog_data['attitude']['q[0]'],
            'ulog_q1': ulog_data['attitude']['q[1]'],
            'ulog_q2': ulog_data['attitude']['q[2]'],
            'ulog_q3': ulog_data['attitude']['q[3]']
        })
        
        # 用線性插值方法將姿態數據同步到位置數據的時間戳
        interp_funcs = {}
        for col in ['ulog_q0', 'ulog_q1', 'ulog_q2', 'ulog_q3']:
            interp_funcs[col] = interpolate.interp1d(
                ulog_att_df['timestamp'], 
                ulog_att_df[col],
                bounds_error=False,
                fill_value="extrapolate"
            )
            
        for col, func in interp_funcs.items():
            ulog_pos_df[col] = func(ulog_pos_df['timestamp'])
    
    # 確定兩個數據集的起始時間偏移
    # 假設兩個數據集的起始時間可能有差異
    # 可以通過交叉相關或手動調整找到最佳的時間偏移
    time_offset = 0  # 默認為0，可能需要調整
    
    # 為EKF數據創建絕對時間戳記
    if 'timestamp' in ekf_df.columns:
        ekf_df['abs_time'] = ekf_df['timestamp']
    else:
        # 如果EKF數據只有相對時間，需要加上基準時間
        ekf_df['abs_time'] = ekf_df['elapsed_time'] + ulog_pos_df['timestamp'].min() + time_offset
    
    # 使用線性插值將ulog數據同步到EKF數據的時間戳
    interp_funcs = {}
    for col in ulog_pos_df.columns:
        if col != 'timestamp':
            interp_funcs[col] = interpolate.interp1d(
                ulog_pos_df['timestamp'], 
                ulog_pos_df[col],
                bounds_error=False,
                fill_value="extrapolate"
            )
    
    # 為每個ulog列創建插值數據
    for col, func in interp_funcs.items():
        ekf_df[col] = func(ekf_df['abs_time'])
    
    print(f"數據同步完成，共有 {len(ekf_df)} 條記錄")
    return ekf_df

def calculate_errors(synced_df):
    """
    計算EKF估計和ulog真值之間的誤差
    """
    print("計算估計誤差...")
    
    # 計算位置誤差
    synced_df['pos_error_x'] = synced_df['pos_x'] - synced_df['ulog_pos_x']
    synced_df['pos_error_y'] = synced_df['pos_y'] - synced_df['ulog_pos_y']
    synced_df['pos_error_z'] = synced_df['pos_z'] - synced_df['ulog_pos_z']
    
    # 計算速度誤差
    synced_df['vel_error_x'] = synced_df['vel_x'] - synced_df['ulog_vel_x']
    synced_df['vel_error_y'] = synced_df['vel_y'] - synced_df['ulog_vel_y']
    synced_df['vel_error_z'] = synced_df['vel_z'] - synced_df['ulog_vel_z']
    
    # 計算姿態誤差 (如果存在四元數數據)
    if all(q in synced_df.columns for q in ['qw', 'qx', 'qy', 'qz']) and \
       all(q in synced_df.columns for q in ['ulog_q0', 'ulog_q1', 'ulog_q2', 'ulog_q3']):
        
        # 計算四元數之間的角度差異
        dot_products = synced_df.apply(
            lambda row: abs(row['qw']*row['ulog_q0'] + row['qx']*row['ulog_q1'] + 
                          row['qy']*row['ulog_q2'] + row['qz']*row['ulog_q3']),
            axis=1
        )
        # 確保點積在[-1, 1]範圍內
        dot_products = dot_products.clip(-1.0, 1.0)
        # 計算角度誤差（弧度）
        synced_df['att_error_rad'] = 2 * np.arccos(dot_products)
        # 轉換為度
        synced_df['att_error_deg'] = synced_df['att_error_rad'] * 180 / np.pi
    
    # 計算整體誤差統計
    pos_rmse = np.sqrt(np.mean(synced_df['pos_error_x']**2 + 
                               synced_df['pos_error_y']**2 + 
                               synced_df['pos_error_z']**2))
    
    vel_rmse = np.sqrt(np.mean(synced_df['vel_error_x']**2 + 
                               synced_df['vel_error_y']**2 + 
                               synced_df['vel_error_z']**2))
    
    att_rmse = np.nan
    if 'att_error_rad' in synced_df.columns:
        att_rmse = np.sqrt(np.mean(synced_df['att_error_rad']**2))
    
    print(f"位置RMSE: {pos_rmse:.3f} m")
    print(f"速度RMSE: {vel_rmse:.3f} m/s")
    if not np.isnan(att_rmse):
        print(f"姿態RMSE: {att_rmse:.3f} rad ({att_rmse*180/np.pi:.3f} deg)")
    
    return synced_df, {
        'pos_rmse': pos_rmse,
        'vel_rmse': vel_rmse,
        'att_rmse': att_rmse
    }

def plot_results(synced_df, errors, output_dir=None):
    """
    繪製比較結果圖表
    """
    print("繪製比較圖表...")
    
    if output_dir is not None:
        os.makedirs(output_dir, exist_ok=True)
    
    # 設置圖形樣式
    plt.style.use('ggplot')
    plt.rcParams['figure.figsize'] = [12, 8]
    plt.rcParams['font.size'] = 12
    
    # 繪製位置比較
    fig, axes = plt.subplots(3, 1, figsize=(12, 14), sharex=True)
    
    axes[0].plot(synced_df['elapsed_time'], synced_df['pos_x'], 'b-', label='EKF估計')
    axes[0].plot(synced_df['elapsed_time'], synced_df['ulog_pos_x'], 'r--', label='PX4參考')
    axes[0].set_ylabel('X位置 (m)')
    axes[0].set_title(f'位置比較 (RMSE: {errors["pos_rmse"]:.3f} m)')
    axes[0].legend()
    
    axes[1].plot(synced_df['elapsed_time'], synced_df['pos_y'], 'b-', label='EKF估計')
    axes[1].plot(synced_df['elapsed_time'], synced_df['ulog_pos_y'], 'r--', label='PX4參考')
    axes[1].set_ylabel('Y位置 (m)')
    axes[1].legend()
    
    axes[2].plot(synced_df['elapsed_time'], synced_df['pos_z'], 'b-', label='EKF估計')
    axes[2].plot(synced_df['elapsed_time'], synced_df['ulog_pos_z'], 'r--', label='PX4參考')
    axes[2].set_ylabel('Z位置 (m)')
    axes[2].set_xlabel('時間 (s)')
    axes[2].legend()
    
    plt.tight_layout()
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'position_comparison.png'), dpi=200)
    
    # 繪製速度比較
    fig, axes = plt.subplots(3, 1, figsize=(12, 14), sharex=True)
    
    axes[0].plot(synced_df['elapsed_time'], synced_df['vel_x'], 'b-', label='EKF估計')
    axes[0].plot(synced_df['elapsed_time'], synced_df['ulog_vel_x'], 'r--', label='PX4參考')
    axes[0].set_ylabel('X速度 (m/s)')
    axes[0].set_title(f'速度比較 (RMSE: {errors["vel_rmse"]:.3f} m/s)')
    axes[0].legend()
    
    axes[1].plot(synced_df['elapsed_time'], synced_df['vel_y'], 'b-', label='EKF估計')
    axes[1].plot(synced_df['elapsed_time'], synced_df['ulog_vel_y'], 'r--', label='PX4參考')
    axes[1].set_ylabel('Y速度 (m/s)')
    axes[1].legend()
    
    axes[2].plot(synced_df['elapsed_time'], synced_df['vel_z'], 'b-', label='EKF估計')
    axes[2].plot(synced_df['elapsed_time'], synced_df['ulog_vel_z'], 'r--', label='PX4參考')
    axes[2].set_ylabel('Z速度 (m/s)')
    axes[2].set_xlabel('時間 (s)')
    axes[2].legend()
    
    plt.tight_layout()
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'velocity_comparison.png'), dpi=200)
    
    # 繪製姿態誤差 (如果存在)
    if 'att_error_deg' in synced_df.columns:
        plt.figure(figsize=(12, 6))
        plt.plot(synced_df['elapsed_time'], synced_df['att_error_deg'], 'g-')
        plt.ylabel('姿態誤差 (度)')
        plt.xlabel('時間 (s)')
        plt.title(f'姿態誤差 (RMSE: {errors["att_rmse"]*180/np.pi:.3f} deg)')
        plt.grid(True)
        plt.tight_layout()
        if output_dir:
            plt.savefig(os.path.join(output_dir, 'attitude_error.png'), dpi=200)
    
    # 繪製3D軌跡對比圖
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(synced_df['pos_x'], synced_df['pos_y'], synced_df['pos_z'], 'b-', 
            label='EKF估計', linewidth=2)
    ax.plot(synced_df['ulog_pos_x'], synced_df['ulog_pos_y'], synced_df['ulog_pos_z'], 
            'r--', label='PX4參考', linewidth=2)
    
    # 設置軸標籤
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D軌跡比較')
    ax.legend()
    
    # 保存圖表
    if output_dir:
        plt.savefig(os.path.join(output_dir, '3d_trajectory.png'), dpi=200)
    
    # 顯示所有圖表 (如果是交互模式)
    if not output_dir:
        plt.show()
    else:
        plt.close('all')

def save_results(synced_df, errors, output_dir):
    """
    保存比較結果到文件
    """
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        
        # 保存同步後的數據
        result_csv = os.path.join(output_dir, 'ekf_ulog_comparison.csv')
        synced_df.to_csv(result_csv, index=False)
        print(f"同步數據已保存到: {result_csv}")
        
        # 保存誤差統計
        with open(os.path.join(output_dir, 'error_stats.txt'), 'w') as f:
            f.write(f"位置RMSE: {errors['pos_rmse']:.6f} m\n")
            f.write(f"速度RMSE: {errors['vel_rmse']:.6f} m/s\n")
            if not np.isnan(errors['att_rmse']):
                f.write(f"姿態RMSE: {errors['att_rmse']:.6f} rad ({errors['att_rmse']*180/np.pi:.6f} deg)\n")

def main():
    parser = argparse.ArgumentParser(description='比較EKF和PX4 ulog數據')
    parser.add_argument('--ekf-log', required=True, help='EKF日誌文件路徑 (CSV格式)')
    parser.add_argument('--ulog', required=True, help='PX4 ulog文件路徑')
    parser.add_argument('--output-dir', help='輸出目錄 (可選)')
    parser.add_argument('--time-offset', type=float, default=0.0, 
                        help='EKF和ulog之間的時間偏移 (秒) (可選)')
    
    args = parser.parse_args()
    
    # 加載數據
    ekf_df = load_ekf_log(args.ekf_log)
    ulog_data = load_ulog_data(args.ulog)
    
    # 同步數據
    synced_df = synchronize_data(ekf_df, ulog_data)
    
    if synced_df is not None:
        # 計算誤差
        synced_df, errors = calculate_errors(synced_df)
        
        # 繪製結果
        plot_results(synced_df, errors, args.output_dir)
        
        # 保存結果
        if args.output_dir:
            save_results(synced_df, errors, args.output_dir)
            print(f"所有結果已保存到: {args.output_dir}")
    
    print("比較完成！")

if __name__ == '__main__':
    main() 