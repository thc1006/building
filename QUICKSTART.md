# 🚀 快速入門指南

使用 NVIDIA Isaac Sim 為赤土崎全齡社福樞紐建立 3D 模型

---

## 📋 前置準備檢查清單

### 硬體需求
- [ ] GPU: NVIDIA RTX 3060 或更高（建議 RTX 4070+）
- [ ] RAM: 32GB 以上
- [ ] 儲存空間: 至少 150GB 可用空間
- [ ] 作業系統: Ubuntu 20.04/22.04 或 Windows 10/11

### 軟體需求
- [ ] NVIDIA 驅動程式（最新版）
- [ ] NVIDIA Isaac Sim 2023.1.1 或更新版本
- [ ] Blender 3.6 或更新版本
- [ ] Python 3.10+

---

## ⚡ 方法A: 使用 Blender 建模（推薦新手）

### 步驟 1: 安裝 Blender

```bash
# Ubuntu
sudo snap install blender --classic

# 或從官網下載
# https://www.blender.org/download/
```

### 步驟 2: 執行建模腳本

**方式 1: 在 Blender 內執行**

1. 開啟 Blender
2. 切換到 **Scripting** 工作區（頂部選單）
3. 點擊 **Open** → 選擇 `/home/user/building/scripts/blender_generate_1F.py`
4. 點擊 **Run Script** (▶️ 按鈕)
5. 等待建模完成（約 1-2 分鐘）

**方式 2: 命令列執行**

```bash
cd /home/user/building
blender --background --python scripts/blender_generate_1F.py
```

### 步驟 3: 檢視模型

在 Blender 中:
1. 按數字鍵 **0** → 切換到攝影機視角
2. 按 **Z** → 選擇 **Rendered** → 查看渲染效果
3. 使用滑鼠中鍵拖曳 → 旋轉視角
4. 滾動滑鼠滾輪 → 縮放

### 步驟 4: 匯出 USD

```
File → Export → Universal Scene Description (.usdc)

設定:
✅ File Format: Binary (.usdc)
✅ Selection Only: OFF
✅ Export Materials: ON

儲存位置: /home/user/building/assets/floor_1F.usdc
```

### 步驟 5: 在 Isaac Sim 中開啟

1. 啟動 Isaac Sim
2. File → Open
3. 選擇 `/home/user/building/assets/floor_1F.usdc`
4. 完成！

---

## ⚡ 方法B: 直接使用 Isaac Sim（進階）

### 步驟 1: 安裝 Isaac Sim

**透過 Omniverse Launcher（推薦）**:

1. 下載 [NVIDIA Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/)
2. 安裝並啟動 Launcher
3. 在 **Exchange** 標籤中找到 **Isaac Sim**
4. 點擊 **Install**（約 15GB 下載）
5. 安裝完成後點擊 **Launch**

### 步驟 2: 執行 Isaac Sim 腳本

**在 Isaac Sim Script Editor 中執行**:

1. 開啟 Isaac Sim
2. Window → Script Editor
3. File → Open Script
4. 選擇 `/home/user/building/scripts/generate_floor_1F.py`
5. 點擊 **Run** (▶️)
6. 等待場景生成

**或使用命令列**:

```bash
# 找到 Isaac Sim 安裝路徑
ISAAC_SIM_PATH=~/.local/share/ov/pkg/isaac_sim-2023.1.1

# 執行腳本
$ISAAC_SIM_PATH/isaac-sim.sh --exec /home/user/building/scripts/generate_floor_1F.py
```

### 步驟 3: 調整場景

在 Isaac Sim 中:
1. **視角控制**:
   - 滑鼠中鍵拖曳: 旋轉
   - Shift + 滑鼠中鍵: 平移
   - 滾輪: 縮放

2. **材質調整**:
   - 在 Stage 面板選擇物件
   - 在 Property 面板調整 Material 參數

3. **照明調整**:
   - 選擇 `/Lights/DomeLight`
   - 調整 Intensity（建議 1000-2000）

---

## 🎨 進階：添加家具

### 使用 Blender 添加簡單家具

在 `blender_generate_1F.py` 腳本的最後添加:

```python
# 添加餐桌範例
def create_table(name, location):
    # 桌面
    bpy.ops.mesh.primitive_cube_add(location=(location[0], location[1], 0.75))
    table_top = bpy.context.active_object
    table_top.name = f"{name}_Top"
    table_top.scale = (0.6, 0.4, 0.025)

    # 桌腳
    for dx, dy in [(0.5, 0.3), (0.5, -0.3), (-0.5, 0.3), (-0.5, -0.3)]:
        bpy.ops.mesh.primitive_cylinder_add(
            location=(location[0] + dx, location[1] + dy, 0.375),
            radius=0.05,
            depth=0.75
        )
        leg = bpy.context.active_object
        leg.name = f"{name}_Leg"

# 在餐廳添加 10 張桌子
for i in range(10):
    row = i // 5
    col = i % 5
    create_table(f"Table_{i+1}", (12 + col * 2, -4 + row * 3, 0))

# 重新執行腳本
```

---

## 🏗️ 建立完整建築物（所有樓層）

### 步驟 1: 建立所有樓層模型

```bash
# 逐一執行每層樓的腳本（尚未建立，需參考 1F 腳本自行修改）
blender --background --python scripts/blender_generate_B1.py
blender --background --python scripts/blender_generate_1F.py
blender --background --python scripts/blender_generate_2F.py
blender --background --python scripts/blender_generate_3F.py
blender --background --python scripts/blender_generate_4F.py
```

### 步驟 2: 組裝完整場景

在 Isaac Sim Script Editor 執行:

```python
from pxr import Usd, UsdGeom, Gf

# 建立新場景
stage = Usd.Stage.CreateNew("/home/user/building/scenes/complete_building.usd")
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# 載入各樓層（設定不同高度）
floors = {
    "B1": -3.5,
    "1F": 0.0,
    "2F": 4.0,
    "3F": 7.8,
    "4F": 11.3,
}

for floor_name, height in floors.items():
    floor_path = f"/Building/{floor_name}"
    floor_xform = UsdGeom.Xform.Define(stage, floor_path)
    floor_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, height))

    # 參照外部 USD 檔案
    floor_ref = stage.DefinePrim(f"{floor_path}/Geometry")
    floor_ref.GetReferences().AddReference(f"../assets/floor_{floor_name}.usdc")

# 儲存場景
stage.Save()
print("✅ 完整建築場景已建立！")
```

---

## 📊 專案檔案結構

建議的專案目錄結構:

```
/home/user/building/
├── CLAUDE.md                          # 建築設計文檔（已存在）
├── QUICKSTART.md                      # 本快速入門指南
├── isaac_sim_implementation_plan.md  # 詳細實作計劃
│
├── scripts/                           # Python 腳本
│   ├── blender_generate_1F.py        # Blender 1F 建模腳本
│   ├── blender_generate_2F.py        # （待建立）
│   ├── blender_generate_3F.py        # （待建立）
│   ├── blender_generate_4F.py        # （待建立）
│   ├── blender_generate_B1.py        # （待建立）
│   └── generate_floor_1F.py          # Isaac Sim 1F 建模腳本
│
├── assets/                            # USD 模型檔案
│   ├── floor_B1.usdc                 # （將由 Blender 匯出）
│   ├── floor_1F.usdc                 # （將由 Blender 匯出）
│   ├── floor_2F.usdc                 # （將由 Blender 匯出）
│   ├── floor_3F.usdc                 # （將由 Blender 匯出）
│   ├── floor_4F.usdc                 # （將由 Blender 匯出）
│   └── furniture/                     # 家具模型
│       ├── tables.usdc
│       ├── chairs.usdc
│       └── equipment.usdc
│
├── scenes/                            # 完整場景檔案
│   ├── complete_building.usd         # 整合所有樓層的場景
│   └── test_1F.usd                   # 測試用場景
│
├── renders/                           # 渲染輸出
│   ├── images/                       # 靜態圖片
│   └── videos/                       # 動畫影片
│
└── blender/                           # Blender 專案檔
    ├── floor_1F.blend
    ├── floor_2F.blend
    └── ...
```

---

## 🎬 渲染與匯出

### 在 Isaac Sim 中渲染高品質圖片

1. 調整到想要的攝影機視角
2. **Render Settings** (右側面板):
   - Renderer: **Path Tracing**
   - Samples: **256** 或更高
   - Denoiser: **ON**

3. **Movie Capture**:
   - Window → Utilities → Movie Capture
   - 設定 Resolution: **1920×1080** 或 **3840×2160**
   - Format: **PNG** (圖片) 或 **MP4** (影片)
   - 點擊 **Capture**

### 匯出 360° 環景圖

```python
# 在 Isaac Sim Script Editor 執行
import omni.kit.viewport.utility as vp_util

# 設定 360° 攝影機
camera = stage.DefinePrim("/Camera360", "Camera")
camera.GetAttribute("focalLength").Set(18)  # 廣角

# 渲染 360° 圖片（需要 360° rendering 插件）
# 或手動旋轉攝影機渲染多張圖片後拼接
```

---

## ❓ 常見問題排解

### Q1: Blender 腳本執行後沒有看到模型？

**A**: 檢查以下幾點:
1. 確認 Console 沒有錯誤訊息（Window → Toggle System Console）
2. 在 Outliner 面板檢查是否有建立物件
3. 按數字鍵 **7** (頂視圖) 和 **Home** 鍵（全畫面顯示）
4. 嘗試縮小視角（滾輪向外滾）

### Q2: Isaac Sim 載入 USD 檔案後畫面全黑？

**A**:
1. 檢查照明: 選擇 `/Lights/DomeLight`，調高 Intensity
2. 調整攝影機位置: 可能在模型內部
3. 切換 Viewport 渲染模式: 右上角選擇不同的渲染模式

### Q3: USD 匯出後材質遺失？

**A**:
1. Blender 匯出時確認勾選 **Export Materials**
2. 使用 **.usdc** 格式（二進位，較穩定）
3. 在 Isaac Sim 中手動重新指定材質

### Q4: 記憶體不足？

**A**:
1. 分樓層建模，不要一次載入全部
2. 降低多邊形數量（Blender: Modifier → Decimate）
3. 使用較低解析度的貼圖

---

## 📚 延伸學習資源

### 官方教學
- [Isaac Sim 官方文檔](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Blender 官方教學](https://www.blender.org/support/tutorials/)
- [USD 入門](https://openusd.org/release/tut_usd_tutorials.html)

### YouTube 推薦
- **Blender 建築建模**: 搜尋 "Blender Architecture Tutorial"
- **Isaac Sim 入門**: [NVIDIA Omniverse Channel](https://www.youtube.com/@NVIDIAOmniverse)

### 社群支援
- [NVIDIA Omniverse 論壇](https://forums.developer.nvidia.com/c/omniverse/)
- [Blender Artists](https://blenderartists.org/)

---

## 🎯 下一步建議

### 本週目標
- [ ] 安裝 Blender 和 Isaac Sim
- [ ] 執行 `blender_generate_1F.py` 建立 1F 模型
- [ ] 成功匯出 USD 並在 Isaac Sim 中開啟
- [ ] 調整材質和照明

### 本月目標
- [ ] 建立所有 5 層樓的模型（參考 1F 腳本）
- [ ] 組裝完整建築場景
- [ ] 添加基本家具
- [ ] 產出高品質渲染圖

### 進階目標
- [ ] 添加動畫（電梯、門、人員移動）
- [ ] 使用 Omniverse 協作功能（多人編輯）
- [ ] 整合物理模擬（碰撞檢測）
- [ ] VR 虛擬實境導覽

---

**祝你建模順利！🚀**

有任何問題歡迎查閱 `isaac_sim_implementation_plan.md` 詳細文檔。
