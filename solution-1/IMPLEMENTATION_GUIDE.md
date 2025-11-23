# 赤土崎全齡社福樞紐 3D 建築生成 - 完整實作指南

## 📋 目錄

1. [系統需求](#系統需求)
2. [環境準備](#環境準備)
3. [快速開始](#快速開始)
4. [詳細實作步驟](#詳細實作步驟)
5. [Claude Code CLI 整合](#claude-code-cli-整合)
6. [進階自訂](#進階自訂)
7. [故障排除](#故障排除)
8. [輸出檔案說明](#輸出檔案說明)

---

## 系統需求

### 硬體需求

- **CPU**: 32-core 或以上（已有 ✓）
- **GPU**: NVIDIA RTX 5090 或更高（已有 ✓）
- **RAM**: 64GB 以上建議
- **儲存空間**: 至少 100GB 可用空間
- **作業系統**: Ubuntu 22.04 LTS 或 Windows 11 with WSL2

### 軟體需求

#### 必要軟體

- **Python**: 3.10 或以上
- **CUDA Toolkit**: 12.1+ (配合 RTX 5090)
- **NVIDIA Driver**: 最新版本 (550+)

#### 選用軟體（3D 生成功能）

- **NVIDIA Isaac Sim**: 5.0.0 或以上
- **NVIDIA Omniverse Launcher**: 最新版

#### Claude 工具

- **Claude Code CLI**: 用於自動化和互動式調整

---

## 環境準備

### 步驟 1: 安裝 NVIDIA Isaac Sim

#### 方法 A: 使用 Omniverse Launcher（推薦）

```bash
# 1. 下載 Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# 2. 執行 Launcher
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# 3. 在 Launcher 中安裝 Isaac Sim
# - 前往 "Exchange" 頁面
# - 搜尋 "Isaac Sim"
# - 點擊 "Install" 安裝 5.0.0 版本
```

#### 方法 B: 使用 Docker 容器

```bash
# 拉取 Isaac Sim Docker 映像
docker pull nvcr.io/nvidia/isaac-sim:4.2.0

# 執行容器
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" --rm --network=host \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  nvcr.io/nvidia/isaac-sim:4.2.0
```

#### 方法 C: 直接從 GitHub 構建（進階）

```bash
# Clone Isaac Sim 原始碼
git clone https://github.com/isaac-sim/IsaacSim.git
cd IsaacSim

# 依照官方文檔構建
# https://docs.isaacsim.omniverse.nvidia.com/5.0.0/build_from_source.html
```

### 步驟 2: 設定環境變數

```bash
# 將以下內容加入 ~/.bashrc 或 ~/.zshrc

# Isaac Sim 路徑（根據你的安裝方式調整）
export ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac-sim-5.0.0"

# 將 Isaac Sim Python 加入 PATH
export PATH="$ISAAC_SIM_PATH:$PATH"

# CUDA 路徑
export CUDA_HOME=/usr/local/cuda-12.1
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# 套用設定
source ~/.bashrc
```

### 步驟 3: 安裝 Claude Code CLI

```bash
# 使用 npm 安裝（需要 Node.js）
npm install -g @anthropic-ai/claude-code

# 或使用 pip 安裝（如果提供 Python 版本）
pip install claude-code-cli

# 驗證安裝
claude-code --version

# 登入
claude-code auth login
```

### 步驟 4: 驗證安裝

```bash
# 檢查 Python
python3 --version  # 應顯示 3.10+

# 檢查 CUDA
nvcc --version  # 應顯示 12.1+

# 檢查 GPU
nvidia-smi  # 應看到 RTX 5090

# 檢查 Isaac Sim（如果已安裝）
$ISAAC_SIM_PATH/isaac-sim.sh --help
```

---

## 快速開始

### 最簡方式（使用自動化腳本）

```bash
# 1. 下載專案檔案
wget https://raw.githubusercontent.com/.../isaac_sim_building_generator.py
wget https://raw.githubusercontent.com/.../claude_code_workflow.sh
chmod +x claude_code_workflow.sh

# 2. 初始化專案
./claude_code_workflow.sh init

# 3. 生成建築資料（不需要 Isaac Sim）
./claude_code_workflow.sh generate-data

# 4. 如果有 Isaac Sim，生成 3D 模型
./claude_code_workflow.sh generate-3d

# 5. 查看結果
ls -lh easterlin-building-3d/output/
```

### 手動執行方式

```bash
# 1. 建立專案目錄
mkdir -p easterlin-building-3d
cd easterlin-building-3d

# 2. 建立 Python 虛擬環境
python3 -m venv venv
source venv/bin/activate

# 3. 安裝相依套件
pip install numpy pyyaml trimesh pillow

# 4. 複製腳本
cp /path/to/isaac_sim_building_generator.py .

# 5a. 執行（不需要 Isaac Sim，僅生成資料）
python isaac_sim_building_generator.py

# 5b. 或在 Isaac Sim 環境中執行（生成 3D）
$ISAAC_SIM_PATH/python.sh isaac_sim_building_generator.py
```

---

## 詳細實作步驟

### 階段 1: 資料準備

#### 1.1 提取建築設計資料

```bash
# 使用 Claude Code 從 Markdown 提取資料
claude-code chat "分析 architectural-floor-plans-2025.md，提取所有樓層的房間資訊、面積、設備清單"

# 手動方式：檢視腳本中的 _initialize_building_data() 方法
```

#### 1.2 結構化資料

腳本會自動將建築資料結構化為：

```python
Building
├── Floor (B1)
│   ├── Room (停車區域)
│   │   ├── 面積: 450 m²
│   │   ├── 位置: (0, 0, -3.5)
│   │   └── 設備: ['parking_spaces', 'lighting', 'cctv']
│   └── ...
├── Floor (1F)
│   └── ...
└── ...
```

### 階段 2: 資料驗證

```bash
# 生成 JSON 資料並檢查
./claude_code_workflow.sh generate-data

# 檢視生成的 JSON
cat easterlin-building-3d/output/json/easterlin_building_data.json | python -m json.tool

# 驗證樓層報告
cat easterlin-building-3d/output/reports/1F_report.txt
```

### 階段 3: 3D 場景生成

#### 3.1 基本 3D 生成

```bash
# 使用自動化腳本
./claude_code_workflow.sh generate-3d

# 或手動執行
cd easterlin-building-3d
$ISAAC_SIM_PATH/python.sh scripts/isaac_sim_building_generator.py
```

#### 3.2 在 Isaac Sim 中檢視

```bash
# 方法 1: 使用 Isaac Sim GUI
$ISAAC_SIM_PATH/isaac-sim.sh

# 在 GUI 中:
# File > Open > 選擇 output/usd/easterlin_building.usd

# 方法 2: 使用命令列
$ISAAC_SIM_PATH/isaac-sim.sh --open output/usd/easterlin_building.usd
```

#### 3.3 調整視角和相機

在 Isaac Sim 中：

1. **調整視角**
   - 滑鼠中鍵拖曳：旋轉視角
   - 滑鼠滾輪：縮放
   - Shift + 中鍵：平移

2. **設定相機**
   - Create > Camera
   - 調整位置對準你想看的樓層
   - 選擇相機 > 右鍵 > Set as Active Camera

3. **截圖**
   - Window > Viewport > Take Screenshot

---

## Claude Code CLI 整合

### 互動式調整

```bash
# 啟動互動模式
./claude_code_workflow.sh interactive

# 或直接使用 Claude Code
cd easterlin-building-3d
claude-code chat
```

### 範例對話

```
你: 我想修改 2F 嬰兒遊戲區的面積為 90 m²

Claude: 我會幫你修改。讓我更新腳本中的相關資料...
[修改 isaac_sim_building_generator.py 中的對應值]

你: 請在 1F 復健訓練室增加「跑步機 3 台」和「復健單槓 2 組」

Claude: 好的，我會在復健訓練室的設備清單中增加這些項目...
[更新設備清單]

你: 重新生成 1F 和 2F 的 3D 模型

Claude: 我會執行生成器僅針對這兩層樓...
[執行部分生成]
```

### 批次處理

```bash
# 建立調整清單
cat > adjustments.txt << EOF
修改 2F 嬰兒室面積為 90 m²
增加 1F 復健訓練室設備: 跑步機 3 台
調整 4F 籃球場高度為 7 公尺
在 3F 新增「遊戲治療室」30 m²
EOF

# 使用 Claude Code 批次處理
while IFS= read -r adjustment; do
    claude-code chat "$adjustment"
done < adjustments.txt

# 重新生成
./claude_code_workflow.sh generate-3d
```

---

## 進階自訂

### 自訂房間顏色

編輯 `isaac_sim_building_generator.py`:

```python
def _create_room_3d(self, floor_path: str, room: Room):
    # 定義房間類型對應的顏色
    room_colors = {
        "parking": [0.6, 0.6, 0.6],        # 灰色
        "activity": [0.9, 0.95, 0.85],     # 淺黃
        "sensory": [0.85, 0.90, 0.95],     # 淺藍
        "dining": [0.95, 0.90, 0.85],      # 淺橘
        "infant_play": [0.95, 0.85, 0.90], # 淺粉
        "counseling": [0.90, 0.85, 0.95],  # 淺紫
        "basketball": [0.85, 0.95, 0.90],  # 淺綠
    }
    
    # 使用房間類型選擇顏色
    wall_color = room_colors.get(room.room_type, [0.95, 0.95, 0.9])
```

### 增加詳細設備模型

```python
def _add_detailed_equipment(self, room_path: str, room: Room):
    """增加詳細設備 3D 模型"""
    
    equipment_models = {
        "wheelchair": self._create_wheelchair_model,
        "table": self._create_table_model,
        "treadmill": self._create_treadmill_model,
    }
    
    for i, equipment in enumerate(room.equipment):
        for key, create_func in equipment_models.items():
            if key in equipment.lower():
                position = [
                    room.position[0] + (i % 3) * 2,
                    room.position[1] + (i // 3) * 2,
                    room.position[2] - room.dimensions[2]/2 + 0.5
                ]
                create_func(f"{room_path}/Equipment_{equipment}", position)

def _create_wheelchair_model(self, prim_path: str, position: List[float]):
    """建立輪椅 3D 模型"""
    # 座椅
    seat = VisualCuboid(
        prim_path=f"{prim_path}/Seat",
        position=position,
        size=[0.5, 0.5, 0.1],
        color=[0.3, 0.3, 0.3]
    )
    # 輪子
    # ... (詳細的輪椅模型)
```

### 匯出其他格式

```python
def export_to_other_formats(self):
    """匯出到其他 3D 格式"""
    
    # 匯出為 glTF
    from pxr import UsdUtils
    UsdUtils.ConvertToGlTF(
        self.stage,
        f"{self.output_dir}/easterlin_building.gltf"
    )
    
    # 匯出為 OBJ（需要額外工具）
    # ...
```

### 增加光照和材質

```python
def _setup_advanced_lighting(self):
    """設置進階光照"""
    
    # 主光源（太陽）
    omni.kit.commands.execute(
        "CreatePrimWithDefaultXform",
        prim_type="DistantLight",
        prim_path="/World/Sun",
        attributes={
            "intensity": 50000,
            "angle": 0.53,
            "color": [1.0, 0.98, 0.95]
        }
    )
    
    # 環境光
    omni.kit.commands.execute(
        "CreatePrimWithDefaultXform",
        prim_type="DomeLight",
        prim_path="/World/Environment",
        attributes={
            "intensity": 1000,
            "texture:file": "path/to/hdri.hdr"
        }
    )
    
    # 室內補光
    for floor in self.building.floors:
        self._add_floor_lighting(floor)
```

---

## 故障排除

### 問題 1: Isaac Sim 無法啟動

```bash
# 檢查 GPU 驅動
nvidia-smi

# 重新安裝驅動
sudo apt update
sudo apt install nvidia-driver-550

# 檢查 CUDA
nvcc --version
```

### 問題 2: Python 模組找不到

```bash
# 確認使用正確的 Python
which python3

# 使用 Isaac Sim 的 Python
$ISAAC_SIM_PATH/python.sh -m pip list

# 重新安裝相依套件
$ISAAC_SIM_PATH/python.sh -m pip install --upgrade numpy
```

### 問題 3: 記憶體不足

```bash
# 減少同時生成的樓層數
# 編輯腳本，分批生成

# 方法 1: 只生成特定樓層
def generate_specific_floors(self, floor_ids: List[str]):
    for floor in self.building.floors:
        if floor.floor_id in floor_ids:
            self.generate_floor_3d(floor)

# 使用
generator.generate_specific_floors(["1F", "2F"])

# 方法 2: 降低模型精度
# 減少設備標記的數量
```

### 問題 4: USD 檔案無法開啟

```bash
# 驗證 USD 檔案
usdcat output/usd/easterlin_building.usd

# 使用 usdview 檢視（如果安裝）
usdview output/usd/easterlin_building.usd

# 重新生成 USD
rm output/usd/*.usd
./claude_code_workflow.sh generate-3d
```

### 問題 5: Claude Code CLI 無回應

```bash
# 檢查網路連線
ping api.anthropic.com

# 重新登入
claude-code auth logout
claude-code auth login

# 檢查 API 配額
claude-code usage
```

---

## 輸出檔案說明

### USD 檔案 (Universal Scene Description)

```
output/usd/easterlin_building.usd
```

- **用途**: 完整的 3D 場景檔案
- **相容軟體**: Isaac Sim, Blender (with USD plugin), Maya, Houdini
- **內容**: 建築幾何、材質、光照、相機設定

### JSON 資料檔案

```
output/json/easterlin_building_data.json
```

- **用途**: 結構化的建築資料
- **內容**: 
  ```json
  {
    "name": "赤土崎全齡社福樞紐",
    "total_area": 3100,
    "floors": [
      {
        "floor_id": "1F",
        "rooms": [...]
      }
    ]
  }
  ```

### 樓層報告

```
output/reports/{FLOOR_ID}_report.txt
```

- **用途**: 人類可讀的樓層詳細資訊
- **內容**: 面積、房間清單、設備統計

### 渲染圖片（手動截圖）

```
output/images/
```

- 從 Isaac Sim 中截圖後手動儲存
- 建議命名: `{FLOOR_ID}_{VIEW}.png`
  - 例如: `1F_overview.png`, `2F_infant_room.png`

---

## 下一步

### 進階功能開發

1. **整合 AI 視覺化**
   - 使用 NVIDIA Cosmos 生成照片級渲染
   - 結合 Edify 3D 生成詳細家具模型

2. **機器人路徑規劃**
   - 在 1F 長照區模擬送餐機器人
   - 在 B1 停車場模擬 AGV

3. **使用者流量模擬**
   - 模擬不同時段的人流
   - 最佳化動線設計

4. **虛擬導覽**
   - 建立互動式 VR 導覽
   - 整合 Web 3D 檢視器

### 持續整合

```bash
# 設定 Git Hooks
# .git/hooks/pre-commit
#!/bin/bash
./claude_code_workflow.sh generate-data
git add output/json/*.json

# 自動化 CI/CD
# .github/workflows/generate-3d.yml
name: Generate 3D Models
on: [push]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Generate 3D
        run: |
          ./claude_code_workflow.sh generate-3d
```

---

## 參考資源

- [NVIDIA Isaac Sim 官方文檔](https://docs.isaacsim.omniverse.nvidia.com/)
- [OpenUSD 文檔](https://openusd.org/docs/index.html)
- [Claude Code CLI 文檔](https://docs.claude.com/claude-code)
- [赤土崎建築設計文檔](https://github.com/32iterations/easterlin-hsinchu/blob/main/docs/design/architectural-floor-plans-2025.md)

## 聯絡支援

- **技術問題**: 透過 Claude Code CLI 提問
- **建築設計問題**: 參考原始 Markdown 文檔
- **Isaac Sim 問題**: NVIDIA Developer Forums

---

**最後更新**: 2025-11-23
**版本**: 1.0.0
**授權**: MIT License
