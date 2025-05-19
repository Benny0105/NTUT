# EKF日誌使用指南

## 概述

本文件說明如何使用PX4風格EKF系統中的日誌功能，幫助您找到並利用系統生成的數據文件。

## 日誌檔案位置

EKF系統會生成以下日誌檔案：

1. **EKF監控日誌** (由`ekf_monitor.py`產生)
   - 檔案位置：`~/Benny/catkin_ws/src/ekf/logs/ekf_monitor_log_YYYYMMDD_HHMMSS.csv`
   - 內容：EKF估計的位置、方向、速度和角速度
   - 格式：CSV檔案，可使用Excel或任何文字編輯器開啟

2. **EKF核心日誌** (由`px4_ekf_simple_node`產生)
   - 檔案位置：`~/Benny/catkin_ws/src/ekf/logs/ekf_log.csv`
   - 內容：更詳細的EKF內部狀態和估計過程

3. **路徑繪製圖像** (由`path_plotter.py`產生)
   - 檔案位置：`~/Benny/catkin_ws/src/ekf/logs/ekf_path.png`
   - 內容：無人機軌跡的視覺化圖像

## 訪問日誌檔案

1. 在系統運行期間，日誌檔案的位置會在終端輸出中顯示：
   ```
   日誌文件: /home/jim/Benny/catkin_ws/src/ekf/logs/ekf_monitor_log_20230324_123456.csv
   ```

2. 您也可以使用以下命令查看最新的日誌檔案：
   ```bash
   ls -ltr ~/Benny/catkin_ws/src/ekf/logs/
   ```

## 日誌檔案格式

EKF監控日誌的列標題為：
```
Time, Position_X, Position_Y, Position_Z, Orientation_X, Orientation_Y, Orientation_Z, Orientation_W, Velocity_X, Velocity_Y, Velocity_Z, Angular_Velocity_X, Angular_Velocity_Y, Angular_Velocity_Z
```

- **Time**：從ROS啟動開始的時間（秒）
- **Position_X/Y/Z**：位置（在NED座標系中為北/東/下）
- **Orientation_X/Y/Z/W**：姿態四元數
- **Velocity_X/Y/Z**：線性速度
- **Angular_Velocity_X/Y/Z**：角速度

## 日誌分析

您可以使用以下工具分析日誌：

1. **Excel/LibreOffice Calc**：開啟CSV檔案並創建圖表
2. **Python with pandas**：用於高級數據分析
3. **MATLAB/Octave**：用於更複雜的處理和視覺化

示例Python腳本：
```python
import pandas as pd
import matplotlib.pyplot as plt

# 讀取CSV文件
df = pd.read_csv('ekf_monitor_log_YYYYMMDD_HHMMSS.csv')

# 繪製位置隨時間變化圖
plt.figure(figsize=(10, 6))
plt.plot(df['Time'], df['Position_X'], label='北')
plt.plot(df['Time'], df['Position_Y'], label='東')
plt.plot(df['Time'], df['Position_Z'], label='下')
plt.xlabel('時間 (秒)')
plt.ylabel('位置 (米)')
plt.legend()
plt.title('EKF估計位置')
plt.grid(True)
plt.savefig('position_plot.png')
plt.show()
```

## 故障排除

如果您找不到日誌檔案：

1. 確認系統運行時沒有錯誤訊息
2. 檢查是否有足夠的磁盤空間
3. 檢查用戶權限是否正確
4. 檢查`~/Benny/catkin_ws/src/ekf/logs/`目錄是否存在 