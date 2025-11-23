# 🏢 赤土崎全齡社福樞紐 - NVIDIA Isaac Sim 3D 模擬專案

## 📋 專案概述
本專案使用 NVIDIA Isaac Sim 建立赤土崎多功能社福設施的 3D 建築模擬，包含完整的 5 層樓（B1+4F）設施配置與人物行為模擬。

### 建築規格
- **總面積**: 3,100 m²
- **樓層**: 地下1層 + 地上4層
- **服務人數**: 140-180人
- **主要功能**:
  - B1: 停車場與設備層（600 m²）
  - 1F: 長照日照中心（800 m²）
  - 2F: 公共托嬰中心（700 m²）
  - 3F: 家庭支持服務（500 m²）
  - 4F: 青少年活動中心（500 m²）

## 🚀 快速開始

### 1. 系統需求
- **GPU**: NVIDIA RTX 3060 以上（推薦 RTX 5090）
- **RAM**: 32GB 以上
- **儲存空間**: 500GB SSD
- **作業系統**: Ubuntu 20.04/22.04 或 Windows 10/11

### 2. 安裝 NVIDIA Isaac Sim

#### 方法一：透過 Omniverse Launcher（推薦）
```bash
# 1. 下載 NVIDIA Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# 2. 執行 Launcher
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# 3. 在 Launcher 中安裝 Isaac Sim 2023.1.1
```

#### 方法二：Docker 容器
```bash
# 拉取 Isaac Sim Docker 映像
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# 執行容器
docker run --gpus all -it \
  -v $(pwd):/workspace \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### 3. 設置專案環境

```bash
# 複製專案
git clone https://github.com/yourusername/easterlin-hsinchu-3d.git
cd easterlin-hsinchu-3d

# 建立專案結構
mkdir -p assets/{architecture,furniture,equipment,characters}
mkdir -p scenes
mkdir -p scripts
mkdir -p configs
mkdir -p simulations
mkdir -p saved_scenes

# 安裝 Python 相依套件
pip install -r requirements.txt
```

### 4. 執行模擬

#### 基本執行
```bash
# 使用預設配置執行
python main_simulation.py

# 使用自訂配置
python main_simulation.py --config configs/building_config.yaml
```

#### 進階選項
```bash
# 無頭模式（不顯示 GUI，用於批次處理）
python main_simulation.py --headless

# 設定模擬時長（秒）
python main_simulation.py --duration 3600

# 載入已儲存的場景
python main_simulation.py --load-scene saved_scenes/scene.usd

# 模擬結束後儲存場景
python main_simulation.py --save-scene
```

## 📁 專案結構

```
easterlin-hsinchu-3d/
├── assets/                 # 3D 模型資產
│   ├── architecture/       # 建築結構模型
│   ├── furniture/         # 家具設施模型
│   ├── equipment/         # 設備器材模型
│   └── characters/        # 人物模型
├── configs/               # 配置檔案
│   └── building_config.yaml
├── scripts/               # Python 腳本
│   ├── building_generator.py      # 建築生成器
│   ├── facility_placer.py        # 設施配置器
│   ├── character_simulation.py   # 人物模擬
│   ├── sensor_system.py          # 感測器系統
│   ├── ai_behavior_system.py     # AI 行為系統
│   └── visualization_dashboard.py # 監控儀表板
├── scenes/                # 場景檔案 (.usd)
├── saved_scenes/          # 儲存的場景
├── docs/                  # 文件
├── main_simulation.py     # 主程式
├── requirements.txt       # Python 相依套件
└── README.md             # 本文件
```

## 🎮 操作指南

### Isaac Sim 內建控制
- **W/A/S/D**: 移動攝影機
- **滑鼠右鍵+拖曳**: 旋轉視角
- **滑鼠滾輪**: 縮放
- **Space**: 播放/暫停模擬
- **F**: 聚焦選中物件
- **G**: 切換網格顯示

### 監控面板功能
模擬執行時會自動開啟監控面板，顯示：
- 各樓層即時使用狀態
- 監視攝影機畫面
- 警報與通知
- 統計資訊

## 🔧 客製化開發

### 新增自訂設施
```python
# 在 scripts/facility_placer.py 中新增
self.asset_library["custom_equipment"] = "/Props/CustomEquipment.usd"
```

### 修改人物行為
```python
# 在 scripts/character_simulation.py 中定義
def custom_behavior(self, character_path):
    # 自訂行為邏輯
    pass
```

### 新增感測器
```python
# 在 scripts/sensor_system.py 中配置
camera = Camera(
    prim_path="/World/Sensors/Cameras/custom_camera",
    frequency=30,
    resolution=(1920, 1080)
)
```

## 📊 數據輸出

模擬會生成以下數據：
- **occupancy_report.csv**: 空間使用率統計
- **interaction_log.json**: 跨齡互動記錄
- **incident_report.txt**: 事件報告
- **simulation_metrics.h5**: 完整模擬數據

## 🐛 常見問題

### Q1: Isaac Sim 無法啟動
確認 GPU 驅動版本：
```bash
nvidia-smi
# 需要 525.60.11 或更新版本
```

### Q2: 記憶體不足錯誤
降低模擬品質：
```python
# 在 main_simulation.py 中調整
simulation_app = SimulationApp({
    "renderer": "PathTracing",  # 改為 "RayTracedLighting"
    "width": 1280,              # 降低解析度
    "height": 720
})
```

### Q3: 模型載入失敗
確認 USD 檔案路徑正確：
```python
# 使用絕對路徑
asset_path = os.path.abspath("assets/model.usd")
```

## 📚 參考資源

- [NVIDIA Isaac Sim 文件](https://docs.omniverse.nvidia.com/isaacsim)
- [USD (Universal Scene Description)](https://openusd.org/)
- [Omniverse 開發指南](https://docs.omniverse.nvidia.com/)
- [建築平面圖原始文件](architectural-floor-plans-2025.md)

## 📧 聯絡資訊

如有問題或建議，歡迎聯絡：
- GitHub Issues: [專案 Issues 頁面](https://github.com/yourusername/easterlin-hsinchu-3d/issues)
- Email: your.email@example.com

## 📄 授權

本專案採用 MIT 授權條款 - 詳見 [LICENSE](LICENSE) 檔案

---

**最後更新**: 2025年11月23日
