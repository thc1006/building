#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - 1F 長照日照中心建模腳本
適用於 NVIDIA Isaac Sim

使用方式:
1. 開啟 Isaac Sim
2. Window → Script Editor
3. 載入並執行此腳本

或使用命令列:
./isaac-sim.sh --exec generate_floor_1F.py
"""

from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf
import omni.usd
import math

# ==================== 配置參數 ====================
FLOOR_NAME = "1F_長照日照中心"
OUTPUT_PATH = "/home/user/building/assets/floor_1F.usd"

# 樓層總尺寸 (根據面積 800 m² 推算為 35m × 23m)
FLOOR_WIDTH = 35.0   # X 軸方向
FLOOR_LENGTH = 23.0  # Y 軸方向
FLOOR_HEIGHT = 4.0   # 樓高

# 牆壁厚度
WALL_THICKNESS = 0.20  # 20 cm

# 顏色定義
COLOR_WALL = Gf.Vec3f(0.95, 0.95, 0.9)     # 米白色
COLOR_FLOOR = Gf.Vec3f(0.85, 0.85, 0.8)    # 淺灰色
COLOR_DEMENTIA = Gf.Vec3f(1.0, 0.9, 0.9)   # 淺粉紅（失智專區）
COLOR_GENERAL = Gf.Vec3f(0.95, 0.98, 1.0)  # 淺藍色（一般日照）

# ==================== 工具函數 ====================

def create_material(stage, name, color):
    """建立 USD 材質"""
    material_path = f"/Materials/{name}"
    material = UsdShade.Material.Define(stage, material_path)

    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    return material

def create_box(stage, path, size, position=(0, 0, 0), color=None):
    """建立立方體"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)

    # 設定變形
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()

    # 縮放
    scale_op = xform.AddScaleOp()
    scale_op.Set(Gf.Vec3d(size[0], size[1], size[2]))

    # 位移
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))

    # 套用材質
    if color:
        material = create_material(stage, f"{path.split('/')[-1]}_mat", color)
        UsdShade.MaterialBindingAPI(cube).Bind(material)

    return cube

def create_wall(stage, name, start_point, end_point, height, thickness=WALL_THICKNESS):
    """建立牆壁（從起點到終點）"""
    # 計算牆壁長度和中心點
    length = math.sqrt((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2)
    center_x = (start_point[0] + end_point[0]) / 2
    center_y = (start_point[1] + end_point[1]) / 2
    center_z = height / 2

    # 計算旋轉角度
    angle = math.atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])
    angle_degrees = math.degrees(angle)

    # 建立牆壁
    wall_path = f"/Building/1F/Walls/{name}"
    wall = UsdGeom.Cube.Define(stage, wall_path)
    wall.CreateSizeAttr(1.0)

    xform = UsdGeom.Xformable(wall)
    xform.ClearXformOpOrder()

    # 先縮放
    scale_op = xform.AddScaleOp()
    scale_op.Set(Gf.Vec3d(length, thickness, height))

    # 再旋轉（繞 Z 軸）
    if angle != 0:
        rotate_op = xform.AddRotateZOp()
        rotate_op.Set(angle_degrees)

    # 最後位移
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(center_x, center_y, center_z))

    # 套用材質
    material = create_material(stage, f"{name}_mat", COLOR_WALL)
    UsdShade.MaterialBindingAPI(wall).Bind(material)

    return wall

def create_room(stage, name, x, y, width, length, height=FLOOR_HEIGHT, color=COLOR_FLOOR):
    """建立房間（地板 + 四面牆）"""
    room_path = f"/Building/1F/Rooms/{name}"

    # 建立房間群組
    UsdGeom.Xform.Define(stage, room_path)

    # 地板
    floor = create_box(
        stage,
        f"{room_path}/Floor",
        size=(width, length, 0.05),
        position=(x, y, 0.025),
        color=color
    )

    # 四面牆
    half_w = width / 2
    half_l = length / 2

    walls = [
        # (name, start_point, end_point)
        ("South", (x - half_w, y - half_l, 0), (x + half_w, y - half_l, 0)),  # 南牆
        ("North", (x - half_w, y + half_l, 0), (x + half_w, y + half_l, 0)),  # 北牆
        ("West", (x - half_w, y - half_l, 0), (x - half_w, y + half_l, 0)),   # 西牆
        ("East", (x + half_w, y - half_l, 0), (x + half_w, y + half_l, 0)),   # 東牆
    ]

    for wall_name, start, end in walls:
        create_wall(stage, f"{name}_{wall_name}", start, end, height)

    print(f"✅ 房間已建立: {name} ({width}m × {length}m)")

    return room_path

# ==================== 主要建模函數 ====================

def build_floor_1F():
    """建立 1F 長照日照中心"""

    print(f"\n{'='*60}")
    print(f"  開始建立: {FLOOR_NAME}")
    print(f"{'='*60}\n")

    # 建立新場景
    stage = Usd.Stage.CreateNew(OUTPUT_PATH)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # 建立根節點
    UsdGeom.Xform.Define(stage, "/Building")
    UsdGeom.Xform.Define(stage, "/Building/1F")
    UsdGeom.Xform.Define(stage, "/Building/1F/Walls")
    UsdGeom.Xform.Define(stage, "/Building/1F/Rooms")
    UsdGeom.Xform.Define(stage, "/Materials")

    # ==================== 外牆 ====================
    print("📝 步驟 1: 建立外牆...")

    half_w = FLOOR_WIDTH / 2
    half_l = FLOOR_LENGTH / 2

    exterior_walls = [
        ("Exterior_South", (-half_w, -half_l, 0), (half_w, -half_l, 0)),
        ("Exterior_North", (-half_w, half_l, 0), (half_w, half_l, 0)),
        ("Exterior_West", (-half_w, -half_l, 0), (-half_w, half_l, 0)),
        ("Exterior_East", (half_w, -half_l, 0), (half_w, half_l, 0)),
    ]

    for name, start, end in exterior_walls:
        create_wall(stage, name, start, end, FLOOR_HEIGHT, thickness=0.30)

    print(f"   外牆尺寸: {FLOOR_WIDTH}m × {FLOOR_LENGTH}m × {FLOOR_HEIGHT}m")

    # ==================== 主地板 ====================
    print("\n📝 步驟 2: 建立主地板...")

    main_floor = create_box(
        stage,
        "/Building/1F/MainFloor",
        size=(FLOOR_WIDTH, FLOOR_LENGTH, 0.05),
        position=(0, 0, 0.025),
        color=COLOR_FLOOR
    )

    # ==================== 失智專區 (200 m²) ====================
    print("\n📝 步驟 3: 建立失智專區 (200 m²)...")

    # 位置: 左側區域
    dementia_x = -10.0  # 左側偏移

    # 1. 安靜活動室 (80 m² → 約 10m × 8m)
    create_room(stage, "失智_安靜活動室", dementia_x, -5, 10, 8, color=COLOR_DEMENTIA)

    # 2. 感官刺激室 (60 m² → 約 8m × 7.5m)
    create_room(stage, "失智_感官刺激室", dementia_x, 5, 8, 7.5, color=COLOR_DEMENTIA)

    # 3. 徘徊走廊（環形，暫以矩形走廊代替）
    # 外圍走廊: 寬度 2m
    corridor_width = 2.0
    # 南側走廊
    create_box(
        stage,
        "/Building/1F/Rooms/失智_徘徊走廊_南",
        size=(12, corridor_width, 0.05),
        position=(dementia_x, -9.5, 0.025),
        color=COLOR_DEMENTIA
    )

    # ==================== 一般日照區 (300 m²) ====================
    print("\n📝 步驟 4: 建立一般日照區 (300 m²)...")

    # 位置: 中央區域
    general_x = 5.0

    # 1. 團體活動室 (150 m² → 15m × 10m)
    create_room(stage, "一般_團體活動室", general_x, -3, 15, 10, color=COLOR_GENERAL)

    # 2. 復健訓練室 (80 m² → 10m × 8m)
    create_room(stage, "一般_復健訓練室", general_x + 8, 6, 10, 8, color=COLOR_GENERAL)

    # 3. 休息室 (70 m² → 10m × 7m)
    create_room(stage, "一般_休息室", general_x - 8, 6, 10, 7, color=COLOR_GENERAL)

    # ==================== 共享空間 (220 m²) ====================
    print("\n📝 步驟 5: 建立共享空間 (220 m²)...")

    # 位置: 右側區域
    shared_x = 12.0

    # 1. 共用餐廳 (120 m² → 12m × 10m)
    create_room(stage, "共用餐廳", shared_x, 0, 12, 10, color=Gf.Vec3f(1.0, 0.98, 0.9))

    # 2. 備餐廚房 (40 m² → 8m × 5m)
    create_room(stage, "備餐廚房", shared_x + 8, 7, 8, 5, color=Gf.Vec3f(0.9, 0.9, 0.85))

    # ==================== 支援空間 (100 m²) ====================
    print("\n📝 步驟 6: 建立支援空間...")

    support_x = -15.0

    # 護理站 (25 m² → 5m × 5m)
    create_room(stage, "護理站", support_x, 0, 5, 5, color=Gf.Vec3f(0.9, 1.0, 0.9))

    # ==================== 添加照明 ====================
    print("\n📝 步驟 7: 建立照明...")

    # 頂部區域光
    dome_light = UsdLux.DomeLight.Define(stage, "/Lights/DomeLight")
    dome_light.CreateIntensityAttr(1000)

    # 室內點光源（每個主要房間一個）
    rooms_to_light = [
        ("失智_安靜活動室", dementia_x, -5),
        ("一般_團體活動室", general_x, -3),
        ("共用餐廳", shared_x, 0),
    ]

    for room_name, lx, ly in rooms_to_light:
        light_path = f"/Lights/{room_name}_Light"
        light = UsdLux.SphereLight.Define(stage, light_path)
        light.CreateIntensityAttr(5000)
        light.CreateRadiusAttr(0.5)

        light_xform = UsdGeom.Xformable(stage.GetPrimAtPath(light_path))
        light_xform.AddTranslateOp().Set(Gf.Vec3d(lx, ly, FLOOR_HEIGHT - 0.5))

    # ==================== 儲存場景 ====================
    print("\n📝 步驟 8: 儲存場景...")
    stage.Save()

    print(f"\n{'='*60}")
    print(f"  ✅ {FLOOR_NAME} 建立完成！")
    print(f"  📁 檔案位置: {OUTPUT_PATH}")
    print(f"{'='*60}\n")

    print("🎬 下一步:")
    print("   1. 開啟 Isaac Sim")
    print(f"   2. File → Open → {OUTPUT_PATH}")
    print("   3. 調整視角、材質、照明")
    print("   4. 添加家具與設備\n")

    return stage

# ==================== 主程式 ====================
if __name__ == "__main__":
    build_floor_1F()
