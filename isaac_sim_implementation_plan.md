# NVIDIA Isaac Sim 建築 3D 建模實作計劃

## 專案概述
**建築名稱**: 赤土崎全齡社福樞紐
**總面積**: 3,100 m²
**樓層**: B1（地下1層）+ 4F（地上4層）
**建築高度**: 約 18-20 公尺
**實作日期**: 2025年11月23日

---

## 📊 建築關鍵尺寸資料

### 樓層尺寸推算

基於總面積 3,100 m² 的分配：

| 樓層 | 面積 | 推算平面尺寸 | 層高 | 累計高度 |
|------|------|------------|------|---------|
| **B1** | 600 m² | 約 30m × 20m | 3.5m | -3.5m |
| **1F** | 800 m² | 約 35m × 23m | 4.0m | 0m (地面層) |
| **2F** | 700 m² | 約 33m × 21m | 3.8m | 4.0m |
| **3F** | 500 m² | 約 28m × 18m | 3.5m | 7.8m |
| **4F** | 500 m² | 約 28m × 18m | 4.0m | 11.3m |
| **屋頂** | - | - | - | 15.3m |

**註**: 實際尺寸需依據建築師的詳細設計調整

---

## 🎯 Isaac Sim 實作策略

### 方案選擇

**推薦方案**: **混合式工作流程**

```
Blender (建模) → USD 匯出 → Isaac Sim (場景組裝、物理模擬、渲染)
```

**原因**:
1. ✅ Isaac Sim 擅長機器人模擬和物理互動，但純建築建模效率較低
2. ✅ Blender 有豐富的建築建模工具（如 Archimesh, BlenderBIM）
3. ✅ USD 格式是 Isaac Sim 的原生格式，完美兼容
4. ✅ 可在 Isaac Sim 中添加互動元素（電梯、門、人員模擬）

---

## 🔧 技術堆疊

### 必要軟體

| 軟體 | 版本 | 用途 |
|------|------|------|
| **NVIDIA Isaac Sim** | 2023.1.1+ | 場景組裝、物理模擬 |
| **Blender** | 3.6+ | 建築建模 |
| **Python** | 3.10+ | Isaac Sim 腳本 |
| **USD** | 23.11+ | 場景描述格式 |

### 選配工具

- **BlenderBIM**: 建築資訊模型（BIM）插件
- **Archimesh**: Blender 建築快速建模插件
- **NVIDIA Omniverse**: 協作平台（可選）

---

## 📐 詳細空間配置資料（按樓層）

### B1 地下層 (600 m²)

#### 主要空間
- **停車區域** (450 m²)
  - 一般停車位: 20格 (2.5m × 5.0m)
  - 無障礙車位: 5格 (3.5m × 5.0m)
  - 親子優先車位: 5格 (2.5m × 5.5m)
  - 車道寬度: 6m

- **設備層** (150 m²)
  - 空調機房: 30 m²
  - 配電室: 20 m²
  - 給排水設備: 25 m²
  - 緊急發電機: 15 m²
  - 倉儲空間: 60 m²

#### 垂直動線
- 客梯1: 2m × 2m
- 客梯2: 2m × 2m
- 安全梯: 1.2m 寬

---

### 1F 一樓：長照日照中心 (800 m²)

#### 主要空間
1. **失智專區** (200 m²)
   - 安靜活動室: 80 m² (約 10m × 8m)
   - 感官刺激室: 60 m² (約 8m × 7.5m)
   - 徘徊走廊: 環形 60 m² (外圍走廊)

2. **一般日照區** (300 m²)
   - 團體活動室: 150 m² (約 15m × 10m, 可分隔為 2×75m²)
   - 復健訓練室: 80 m² (約 10m × 8m)
   - 休息室: 70 m² (約 10m × 7m)

3. **共享空間** (220 m²)
   - 共用餐廳: 120 m² (約 12m × 10m)
   - 備餐廚房: 40 m² (約 8m × 5m)
   - 無障礙庭園: 60 m² (戶外)

4. **支援空間** (100 m²)
   - 護理站: 25 m²
   - 無障礙廁所: 40 m² (6間)
   - 儲藏室: 20 m²
   - 員工休息室: 15 m²

#### 關鍵設計參數
- 走道寬度: 120cm 以上（輪椅通行）
- 門寬: 90cm 以上
- 天花板高: 3.5-4.0m
- 隔音牆: STC 65（失智專區）

---

### 2F 二樓：公共托嬰中心 (700 m²)

#### 主要空間
1. **嬰兒室 0-1歲** (180 m²)
   - 嬰兒遊戲區: 80 m²
   - 嬰兒午睡室: 60 m² (15張嬰兒床，床距60cm)
   - 調乳室: 20 m²
   - 尿布更換區: 20 m²

2. **幼兒室 1-2歲** (250 m²)
   - 幼兒遊戲區: 120 m² (約 12m × 10m)
   - 幼兒午睡室: 80 m² (30張睡墊，床距50cm)
   - 閱讀角: 30 m²
   - 感覺統合區: 20 m²

3. **共享設施** (170 m²)
   - 幼兒餐廳: 60 m²
   - 備餐區: 25 m²
   - 戶外遊戲區: 85 m² (陽台)

4. **支援空間** (100 m²)
   - 行政辦公室: 25 m²
   - 保健室: 15 m²
   - 親子廁所: 30 m²
   - 教材儲藏室: 20 m²
   - 教保員休息室: 10 m²

#### 關鍵設計參數
- 地板: IIC 65（減少衝擊噪音）
- 隔音: STC 60-65
- 天花板高: 3.2-3.8m
- 安全設施: 軟包牆角、防夾手門擋

---

### 3F 三樓：家庭支持服務中心 (500 m²)

#### 主要空間
1. **諮商服務區** (150 m²)
   - 個別諮商室: 4間 × 15 m² = 60 m²
   - 家族治療室: 40 m²
   - 遊戲治療室: 30 m²
   - 等候區: 20 m²

2. **親職教育區** (180 m²)
   - 多功能教室: 100 m² (約 12.5m × 8m)
   - 親子烹飪教室: 50 m² (中島式)
   - 手作工作坊: 30 m²

3. **社區營造區** (100 m²)
   - 社區共餐廚房: 60 m² (營業級)
   - 志工培訓室: 40 m²

4. **支援空間** (70 m²)
   - 社工辦公室: 30 m²
   - 教材室: 20 m²
   - 廁所: 20 m²

#### 關鍵設計參數
- 諮商室隔音: STC 60
- 多功能教室: 可容納50人（講座模式）
- 廚房: 符合營業級標準（排煙、三槽洗碗）

---

### 4F 四樓：青少年活動中心 (500 m²)

#### 主要空間
1. **運動休閒區** (220 m²)
   - 室內籃球場: 150 m² (15m × 10m, 半場)
   - 舞蹈/韻律教室: 50 m² (鏡面牆)
   - 體適能區: 20 m²

2. **學習創作區** (150 m²)
   - 自習室: 60 m² (40個座位)
   - 電腦教室: 50 m² (20台電腦)
   - 創客空間: 40 m² (3D列印、Arduino)

3. **社交娛樂區** (80 m²)
   - 交誼廳: 50 m² (桌遊、Switch)
   - 團練室: 30 m² (樂團排練)

4. **支援空間** (50 m²)
   - 輔導辦公室: 20 m²
   - 廁所/更衣室: 20 m²
   - 器材室: 10 m²

#### 關鍵設計參數
- 籃球場天花板高: 6m 以上
- 地板: IIC 70（減少對下層衝擊）
- 團練室: STC 65 隔音
- 天花板高: 4.0m（運動區需較高）

---

## 🛠️ 實作步驟

### 階段1: 環境準備 (1-2天)

#### 1.1 安裝 Isaac Sim

```bash
# 方法1: Omniverse Launcher (推薦)
# 1. 下載 NVIDIA Omniverse Launcher
# 2. 安裝 Isaac Sim 2023.1.1+

# 方法2: Docker (進階)
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
```

#### 1.2 安裝 Blender

```bash
# Ubuntu
sudo snap install blender --classic

# 或從官網下載
wget https://www.blender.org/download/
```

#### 1.3 安裝 Python 依賴

```bash
pip install numpy pxr usd-core
```

---

### 階段2: Blender 建模 (每層樓 2-3天)

#### 2.1 建立基礎結構

**工作流程**:

```
1. 新增 Blender 專案
2. 設定單位: 公制 (m)
3. 建立樓層平面
4. 擠出牆壁 (厚度 20-30cm)
5. 建立門窗開口
6. 分配材質
```

**Blender 快捷操作**:

```python
# Blender Python 腳本範例: 建立 1F 平面
import bpy

# 清空場景
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# 建立地板
bpy.ops.mesh.primitive_plane_add(size=1, location=(0, 0, 0))
floor = bpy.context.active_object
floor.scale = (35, 23, 1)  # 1F: 35m × 23m
floor.name = "Floor_1F"

# 建立外牆
def create_wall(name, location, scale):
    bpy.ops.mesh.primitive_cube_add(location=location)
    wall = bpy.context.active_object
    wall.scale = scale
    wall.name = name
    return wall

# 南牆
create_wall("Wall_South", (0, -11.5, 2), (35, 0.3, 4))
# 北牆
create_wall("Wall_North", (0, 11.5, 2), (35, 0.3, 4))
# 東牆
create_wall("Wall_East", (17.5, 0, 2), (0.3, 23, 4))
# 西牆
create_wall("Wall_West", (-17.5, 0, 2), (0.3, 23, 4))
```

#### 2.2 建立內部空間（以 1F 為例）

**分區建模策略**:

```
1F 空間分區:
├── 失智專區 (200 m²) → 左側區域
│   ├── 安靜活動室 (10m × 8m)
│   ├── 感官刺激室 (8m × 7.5m)
│   └── 徘徊走廊 (環形)
├── 一般日照區 (300 m²) → 中央區域
│   ├── 團體活動室 (15m × 10m)
│   ├── 復健訓練室 (10m × 8m)
│   └── 休息室 (10m × 7m)
└── 共享空間 (220 m²) → 右側+後方
    ├── 共用餐廳 (12m × 10m)
    ├── 備餐廚房 (8m × 5m)
    └── 無障礙庭園 (戶外)
```

**Blender 分區建模**:

```python
# 建立隔間牆 (失智專區)
def create_room_walls(room_name, x, y, width, length, height=4.0):
    """建立房間的四面牆"""
    wall_thickness = 0.2

    # 地板
    bpy.ops.mesh.primitive_plane_add(location=(x, y, 0))
    floor = bpy.context.active_object
    floor.scale = (width/2, length/2, 1)
    floor.name = f"{room_name}_Floor"

    # 四面牆
    walls = [
        (f"{room_name}_Wall_S", (x, y - length/2, height/2), (width, wall_thickness, height)),
        (f"{room_name}_Wall_N", (x, y + length/2, height/2), (width, wall_thickness, height)),
        (f"{room_name}_Wall_E", (x + width/2, y, height/2), (wall_thickness, length, height)),
        (f"{room_name}_Wall_W", (x - width/2, y, height/2), (wall_thickness, length, height)),
    ]

    for name, loc, scale in walls:
        create_wall(name, loc, scale)

# 使用範例
create_room_walls("失智_安靜活動室", -10, -5, 10, 8)
create_room_walls("失智_感官刺激室", -10, 5, 8, 7.5)
```

#### 2.3 添加設施與家具

**設施清單**（1F 為例）:

```
失智專區:
□ 可調高度桌椅 ×10組
□ 懷舊治療展示櫃 ×2
□ 音樂治療設備
□ 感官刺激設備（泡泡管、光纖燈）

一般日照區:
□ 團體活動桌椅 ×60張
□ 投影設備
□ 復健器材（跑步機×2、腳踏車×3）

共用餐廳:
□ 餐桌 ×10組（6人座）
□ 輪椅友善桌 ×3組
□ 保溫餐車 ×3台
```

**Blender 家具建模**:

```python
# 簡單家具範例
def create_table(name, location, size=(1.2, 0.8, 0.75)):
    """建立桌子"""
    # 桌面
    bpy.ops.mesh.primitive_cube_add(location=(location[0], location[1], size[2]))
    table_top = bpy.context.active_object
    table_top.scale = (size[0]/2, size[1]/2, 0.05)
    table_top.name = f"{name}_Top"

    # 桌腳 (4隻)
    leg_positions = [
        (location[0] + size[0]/2 - 0.1, location[1] + size[1]/2 - 0.1, size[2]/2),
        (location[0] + size[0]/2 - 0.1, location[1] - size[1]/2 + 0.1, size[2]/2),
        (location[0] - size[0]/2 + 0.1, location[1] + size[1]/2 - 0.1, size[2]/2),
        (location[0] - size[0]/2 + 0.1, location[1] - size[1]/2 + 0.1, size[2]/2),
    ]

    for i, pos in enumerate(leg_positions):
        bpy.ops.mesh.primitive_cylinder_add(location=pos, radius=0.05, depth=size[2])
        leg = bpy.context.active_object
        leg.name = f"{name}_Leg{i+1}"

# 批量建立餐廳桌椅
for i in range(10):
    row = i // 5
    col = i % 5
    create_table(f"Dining_Table_{i+1}", (col * 2.5, row * 3, 0))
```

#### 2.4 匯出 USD

**Blender → USD 匯出設定**:

```
File → Export → Universal Scene Description (.usd/.usdc/.usda)

設定:
✅ Selection Only: OFF (匯出所有物件)
✅ Visible Only: ON
✅ Export Materials: ON
✅ Export Subdivision: OFF
✅ File Format: Binary (.usdc) ← 推薦，檔案較小

儲存位置: /home/user/building/assets/floor_1F.usdc
```

**每層樓分別匯出**:

```
/home/user/building/assets/
├── floor_B1.usdc   (B1地下層)
├── floor_1F.usdc   (1F長照中心)
├── floor_2F.usdc   (2F托嬰中心)
├── floor_3F.usdc   (3F家庭支持)
├── floor_4F.usdc   (4F青少年中心)
└── furniture/
    ├── tables.usdc
    ├── chairs.usdc
    └── equipment.usdc
```

---

### 階段3: Isaac Sim 場景組裝 (2-3天)

#### 3.1 建立 Isaac Sim 專案

**啟動 Isaac Sim**:

```bash
# 方法1: Omniverse Launcher
# 開啟 Isaac Sim → Create New Stage

# 方法2: 命令列
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh
```

#### 3.2 Python 腳本組裝場景

**完整範例腳本**: `build_complete_scene.py`

```python
#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - Isaac Sim 場景生成腳本
生成日期: 2025-11-23
"""

import omni
from pxr import Usd, UsdGeom, Gf, UsdPhysics, UsdShade
import numpy as np

# ===== 配置參數 =====
BUILDING_NAME = "赤土崎全齡社福樞紐"
USD_ASSETS_PATH = "/home/user/building/assets"
OUTPUT_SCENE_PATH = "/home/user/building/scenes/complete_building.usd"

# 樓層高度配置 (累計高度)
FLOOR_HEIGHTS = {
    "B1": -3.5,
    "1F": 0.0,
    "2F": 4.0,
    "3F": 7.8,
    "4F": 11.3,
}

# ===== 場景初始化 =====
def create_stage():
    """建立新的 USD Stage"""
    stage = Usd.Stage.CreateNew(OUTPUT_SCENE_PATH)

    # 設定場景單位為公尺
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    return stage

# ===== 載入樓層 =====
def load_floor(stage, floor_name, height):
    """載入單一樓層的 USD 檔案並設定位置"""

    # 建立樓層群組
    floor_xform_path = f"/Building/{floor_name}"
    floor_xform = UsdGeom.Xform.Define(stage, floor_xform_path)

    # 設定樓層高度
    floor_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, height))

    # 參照外部 USD 檔案
    floor_usd_path = f"{USD_ASSETS_PATH}/floor_{floor_name}.usdc"
    floor_ref = stage.DefinePrim(f"{floor_xform_path}/Geometry")
    floor_ref.GetReferences().AddReference(floor_usd_path)

    print(f"✅ 載入樓層: {floor_name} (高度: {height}m)")

    return floor_xform

# ===== 建立照明 =====
def create_lighting(stage):
    """建立場景照明"""

    # 環境光（HDRI）
    dome_light_path = "/Environment/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
    dome_light.CreateIntensityAttr(1000)
    dome_light.CreateTextureFileAttr("./textures/sky.hdr")  # 可選

    # 太陽光
    sun_light_path = "/Environment/SunLight"
    sun_light = UsdLux.DistantLight.Define(stage, sun_light_path)
    sun_light.CreateIntensityAttr(3000)
    sun_xform = UsdGeom.Xformable(stage.GetPrimAtPath(sun_light_path))
    sun_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 135, 0))

    print("✅ 照明建立完成")

# ===== 建立攝影機視角 =====
def create_cameras(stage):
    """建立多個預設攝影機視角"""

    cameras = {
        "Overview": {
            "position": (50, -50, 30),
            "target": (0, 0, 6),
        },
        "1F_Entrance": {
            "position": (0, -20, 1.6),
            "target": (0, 0, 1.6),
        },
        "2F_Nursery": {
            "position": (10, -15, 5),
            "target": (0, 0, 4.5),
        },
        "4F_Basketball": {
            "position": (0, -10, 14),
            "target": (0, 5, 12),
        },
    }

    for cam_name, cam_params in cameras.items():
        cam_path = f"/Cameras/{cam_name}"
        camera = UsdGeom.Camera.Define(stage, cam_path)

        # 設定攝影機位置
        cam_xform = UsdGeom.Xformable(stage.GetPrimAtPath(cam_path))
        cam_xform.AddTranslateOp().Set(Gf.Vec3d(*cam_params["position"]))

        # 設定焦距
        camera.CreateFocalLengthAttr(50)

        print(f"✅ 攝影機建立: {cam_name}")

# ===== 添加物理屬性 =====
def add_physics(stage):
    """為建築物添加物理屬性（碰撞檢測）"""

    # 建立物理場景
    physics_scene_path = "/PhysicsScene"
    physics_scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    # 為每個樓層添加靜態碰撞器
    for floor_name in FLOOR_HEIGHTS.keys():
        floor_path = f"/Building/{floor_name}/Geometry"
        floor_prim = stage.GetPrimAtPath(floor_path)

        if floor_prim.IsValid():
            # 添加碰撞 API
            UsdPhysics.CollisionAPI.Apply(floor_prim)

            # 設為靜態物體
            UsdPhysics.RigidBodyAPI.Apply(floor_prim)
            rigid_body = UsdPhysics.RigidBodyAPI(floor_prim)
            rigid_body.CreateRigidBodyEnabledAttr(False)  # 靜態

    print("✅ 物理屬性添加完成")

# ===== 主程式 =====
def main():
    """主要執行流程"""

    print(f"\n{'='*60}")
    print(f"  {BUILDING_NAME} - Isaac Sim 場景生成")
    print(f"{'='*60}\n")

    # 1. 建立場景
    print("📝 步驟 1: 建立 USD Stage...")
    stage = create_stage()

    # 2. 載入所有樓層
    print("\n📝 步驟 2: 載入建築樓層...")
    for floor_name, height in FLOOR_HEIGHTS.items():
        load_floor(stage, floor_name, height)

    # 3. 建立照明
    print("\n📝 步驟 3: 建立照明...")
    create_lighting(stage)

    # 4. 建立攝影機
    print("\n📝 步驟 4: 建立攝影機視角...")
    create_cameras(stage)

    # 5. 添加物理屬性
    print("\n📝 步驟 5: 添加物理屬性...")
    add_physics(stage)

    # 6. 儲存場景
    print("\n📝 步驟 6: 儲存場景...")
    stage.Save()
    print(f"✅ 場景已儲存至: {OUTPUT_SCENE_PATH}")

    print(f"\n{'='*60}")
    print("  ✨ 場景生成完成！")
    print(f"{'='*60}\n")

    print("🎬 下一步:")
    print("   1. 開啟 Isaac Sim")
    print(f"   2. File → Open → {OUTPUT_SCENE_PATH}")
    print("   3. 調整材質、光照、攝影機")
    print("   4. 渲染或匯出影片\n")

if __name__ == "__main__":
    main()
```

**執行腳本**:

```bash
cd /home/user/building
python build_complete_scene.py
```

#### 3.3 在 Isaac Sim 中開啟場景

```
1. 啟動 Isaac Sim
2. File → Open
3. 選擇: /home/user/building/scenes/complete_building.usd
4. 等待載入完成
```

---

### 階段4: 進階功能 (選配)

#### 4.1 添加互動元素

**電梯動畫**:

```python
# 在 Isaac Sim Script Editor 中執行
import omni.timeline
from pxr import Gf

# 取得電梯物件
stage = omni.usd.get_context().get_stage()
elevator_path = "/Building/1F/Elevator1"
elevator = stage.GetPrimAtPath(elevator_path)

# 建立動畫（1F → 4F）
elevator_xform = UsdGeom.Xformable(elevator)
translate_op = elevator_xform.AddTranslateOp()

# 設定關鍵影格
translate_op.Set(Gf.Vec3d(0, 0, 0), time=0)      # 1F
translate_op.Set(Gf.Vec3d(0, 0, 4), time=50)     # 2F
translate_op.Set(Gf.Vec3d(0, 0, 7.8), time=100)  # 3F
translate_op.Set(Gf.Vec3d(0, 0, 11.3), time=150) # 4F
```

#### 4.2 人員模擬

**使用 NVIDIA Character Generator** (選配):

```
1. 訪問: https://www.nvidia.com/en-us/omniverse/apps/character-creator/
2. 建立角色模型（長者、幼兒、青少年、照服員）
3. 匯入 Isaac Sim
4. 使用 Isaac Sim 的 Crowd Simulation 功能
```

#### 4.3 渲染與匯出

**高品質渲染設定**:

```
Render Settings (視窗右側):
├── Renderer: Path Tracing (最高品質)
├── Samples per Pixel: 256+
├── Max Bounces: 8
├── Resolution: 1920×1080 或 3840×2160
└── Denoiser: ON
```

**匯出影片**:

```
Window → Utilities → Movie Capture

設定:
├── Frame Range: 0-300 (10秒 @ 30fps)
├── Resolution: 1920×1080
├── Format: MP4
└── Output Path: /home/user/building/renders/building_tour.mp4
```

---

## 📊 專案時程估算

| 階段 | 任務 | 預估時間 |
|------|------|---------|
| **1** | 環境準備、軟體安裝 | 1-2 天 |
| **2** | Blender 建模 (5層樓) | 10-15 天 |
| | - B1 停車場 | 2 天 |
| | - 1F 長照中心 | 3 天 |
| | - 2F 托嬰中心 | 3 天 |
| | - 3F 家庭支持 | 2 天 |
| | - 4F 青少年中心 | 2 天 |
| **3** | Isaac Sim 場景組裝 | 2-3 天 |
| **4** | 材質、光照調整 | 2-3 天 |
| **5** | 互動功能、動畫 (選配) | 3-5 天 |
| **6** | 渲染、匯出 | 1-2 天 |
| **總計** | | **19-30 天** |

**建議**: 採用迭代開發，先完成 1F 的完整流程作為原型，確認工作流程後再擴展到其他樓層。

---

## 🔍 替代方案

### 方案B: 純 Isaac Sim 建模（不推薦）

如果不使用 Blender，可以完全用 Isaac Sim 的 Python API 建模：

```python
from pxr import UsdGeom, Gf

def create_room_in_isaac(stage, name, width, length, height):
    """在 Isaac Sim 中直接建立房間"""

    # 建立地板
    floor_path = f"/Building/{name}/Floor"
    floor = UsdGeom.Mesh.Define(stage, floor_path)

    # 定義頂點
    points = [
        Gf.Vec3f(-width/2, -length/2, 0),
        Gf.Vec3f(width/2, -length/2, 0),
        Gf.Vec3f(width/2, length/2, 0),
        Gf.Vec3f(-width/2, length/2, 0),
    ]
    floor.GetPointsAttr().Set(points)

    # 定義面（四邊形）
    face_vertex_counts = [4]
    face_vertex_indices = [0, 1, 2, 3]
    floor.GetFaceVertexCountsAttr().Set(face_vertex_counts)
    floor.GetFaceVertexIndicesAttr().Set(face_vertex_indices)

    # ... 建立牆壁（類似邏輯）

# 使用
stage = Usd.Stage.CreateNew("/home/user/building/test.usd")
create_room_in_isaac(stage, "TestRoom", 10, 8, 4)
stage.Save()
```

**缺點**:
- ❌ 程式碼冗長
- ❌ 難以建立複雜幾何
- ❌ 沒有視覺化即時預覽

### 方案C: 使用 SketchUp + USD 外掛

```
SketchUp (建模) → USD Exporter → Isaac Sim
```

**優點**:
- ✅ SketchUp 對建築設計師更友善
- ✅ 有大量建築元件庫

**缺點**:
- ❌ SketchUp Pro 需要付費
- ❌ USD 外掛可能不穩定

---

## 📚 參考資源

### 官方文件
- **Isaac Sim 文件**: https://docs.omniverse.nvidia.com/isaacsim/latest/
- **USD 文件**: https://openusd.org/release/index.html
- **Blender 文件**: https://docs.blender.org/

### 教學資源
- **Isaac Sim 建築範例**: [Omniverse Samples](https://docs.omniverse.nvidia.com/isaacsim/latest/samples.html)
- **Blender 建築建模**: [BlenderGuru - Architecture](https://www.blenderguru.com/)
- **USD 入門**: [Pixar USD Tutorials](https://openusd.org/release/tut_usd_tutorials.html)

### 社群論壇
- **NVIDIA Omniverse 論壇**: https://forums.developer.nvidia.com/c/omniverse/
- **Blender Artists**: https://blenderartists.org/

---

## 🎯 下一步行動

### 立即可做
1. ✅ **安裝環境**: 下載 Isaac Sim、Blender
2. ✅ **建立原型**: 先建立 1F 的簡化版本（只有主要空間）
3. ✅ **測試流程**: Blender → USD → Isaac Sim

### 本週目標
- 完成 1F 長照中心的完整建模
- 測試 USD 匯出/匯入流程
- 在 Isaac Sim 中調整材質和照明

### 本月目標
- 完成所有 5 層樓的建模
- 組裝完整場景
- 產出渲染影片

---

## ❓ 常見問題

### Q1: Isaac Sim 對硬體要求？
**A**:
- GPU: NVIDIA RTX 3060 以上（建議 RTX 4070+）
- RAM: 32GB 以上
- 儲存空間: 50GB+（安裝）+ 100GB+（專案）
- OS: Ubuntu 20.04/22.04 或 Windows 10/11

### Q2: Blender 建模需要多久學習？
**A**:
- 基礎操作: 1-2 週
- 建築建模: 1 個月（搭配 Archimesh 插件可加速）
- 進階技巧: 3-6 個月

### Q3: 可以團隊協作嗎？
**A**:
可以！使用 **NVIDIA Omniverse Nucleus** 伺服器:
```
1. 設定 Nucleus Server（本地或雲端）
2. 所有成員連接同一伺服器
3. 即時協作編輯 USD 場景
```

### Q4: 如何優化效能？
**A**:
- 使用 LOD (Level of Detail): 遠距離物件使用低精度模型
- 實例化 (Instancing): 重複物件（如椅子）使用實例參照
- 材質優化: 降低貼圖解析度（4K → 2K）
- 分層載入: 依需求載入特定樓層

---

**文件版本**: v1.0
**更新日期**: 2025-11-23
**作者**: Claude
**專案狀態**: 規劃中
