#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - 完整場景組裝腳本
組合所有樓層 (B1 + 1F-4F) 成為完整建築

執行環境: NVIDIA Isaac Sim

使用方式:
方法1: Isaac Sim Script Editor
  1. 開啟 Isaac Sim
  2. Window → Script Editor
  3. 載入並執行此腳本

方法2: 命令列
  ~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh \
    --exec /home/user/building/scripts/assemble_complete_building.py
"""

from pxr import Usd, UsdGeom, Gf, UsdLux, UsdPhysics
import omni.usd
import os

# ==================== 配置參數 ====================
BUILDING_NAME = "赤土崎全齡社福樞紐"
ASSETS_PATH = "/home/user/building/assets"
OUTPUT_SCENE = "/home/user/building/scenes/complete_building.usd"

# 樓層高度配置（累計高度，單位：公尺）
FLOOR_HEIGHTS = {
    "B1": -3.5,   # 地下1層
    "1F": 0.0,    # 1樓（地面層）
    "2F": 4.0,    # 2樓
    "3F": 7.8,    # 3樓
    "4F": 11.3,   # 4樓
}

# 樓層資訊
FLOOR_INFO = {
    "B1": {"name": "停車場與設備層", "area": 600, "color": (0.6, 0.6, 0.6)},
    "1F": {"name": "長照日照中心", "area": 800, "color": (0.9, 0.95, 1.0)},
    "2F": {"name": "公共托嬰中心", "area": 700, "color": (1.0, 0.95, 0.95)},
    "3F": {"name": "家庭支持服務中心", "area": 500, "color": (0.95, 0.98, 1.0)},
    "4F": {"name": "青少年活動中心", "area": 500, "color": (1.0, 0.98, 0.9)},
}

# ==================== 初始化場景 ====================
def create_stage():
    """建立新的 USD Stage"""
    print(f"\n{'='*70}")
    print(f"  {BUILDING_NAME} - 完整場景組裝")
    print(f"{'='*70}\n")

    # 建立新場景
    stage = Usd.Stage.CreateNew(OUTPUT_SCENE)

    # 設定場景單位與方向
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    print(f"✅ 場景已建立: {OUTPUT_SCENE}")
    return stage

# ==================== 載入樓層 ====================
def load_floor(stage, floor_id):
    """載入單一樓層的 USD 檔案並設定位置"""

    floor_name = FLOOR_INFO[floor_id]["name"]
    height = FLOOR_HEIGHTS[floor_id]
    area = FLOOR_INFO[floor_id]["area"]

    print(f"\n📝 載入樓層: {floor_id} - {floor_name}")
    print(f"   高度: {height}m | 面積: {area} m²")

    # 建立樓層群組
    floor_xform_path = f"/Building/{floor_id}"
    floor_xform = UsdGeom.Xform.Define(stage, floor_xform_path)

    # 設定樓層高度（位移）
    floor_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, height))

    # 參照外部 USD 檔案
    floor_usd_file = f"{ASSETS_PATH}/floor_{floor_id}.usdc"

    # 檢查檔案是否存在
    if not os.path.exists(floor_usd_file):
        print(f"   ⚠️  警告: USD 檔案不存在: {floor_usd_file}")
        print(f"   → 請先執行 blender_generate_{floor_id}.py 並匯出 USD")
        return None

    # 添加參照
    floor_ref_path = f"{floor_xform_path}/Geometry"
    floor_ref = stage.DefinePrim(floor_ref_path)
    floor_ref.GetReferences().AddReference(floor_usd_file)

    print(f"   ✅ 已載入: {floor_usd_file}")

    return floor_xform

# ==================== 建立環境照明 ====================
def create_lighting(stage):
    """建立場景照明"""
    print(f"\n📝 建立環境照明...")

    # 1. 環境光（HDRI Dome Light）
    dome_light_path = "/Environment/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
    dome_light.CreateIntensityAttr(1500)
    dome_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    # 2. 太陽光（Distant Light）
    sun_light_path = "/Environment/SunLight"
    sun_light = UsdLux.DistantLight.Define(stage, sun_light_path)
    sun_light.CreateIntensityAttr(3000)
    sun_light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
    sun_light.CreateAngleAttr(0.53)  # 太陽角度

    # 設定太陽方向（東南方，高度45度）
    sun_xform = UsdGeom.Xformable(stage.GetPrimAtPath(sun_light_path))
    sun_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 135, 0))

    print(f"   ✅ 環境光已建立")
    print(f"   ✅ 太陽光已建立")

# ==================== 建立攝影機 ====================
def create_cameras(stage):
    """建立多個預設攝影機視角"""
    print(f"\n📝 建立攝影機視角...")

    cameras = {
        "Overview_Aerial": {
            "description": "鳥瞰全景",
            "position": (60, -50, 40),
            "rotation": (-55, 0, 40),
            "focal_length": 35,
        },
        "Front_Entrance": {
            "description": "1F 正面入口",
            "position": (0, -30, 2),
            "rotation": (-85, 0, 0),
            "focal_length": 50,
        },
        "1F_Interior": {
            "description": "1F 長照中心內部",
            "position": (0, -15, 1.6),
            "rotation": (-80, 0, 0),
            "focal_length": 35,
        },
        "2F_Nursery": {
            "description": "2F 托嬰中心",
            "position": (10, -20, 5),
            "rotation": (-75, 0, 20),
            "focal_length": 40,
        },
        "4F_Basketball": {
            "description": "4F 籃球場",
            "position": (0, -15, 14),
            "rotation": (-80, 0, 0),
            "focal_length": 30,
        },
        "Side_Elevation": {
            "description": "側面立面",
            "position": (50, 0, 8),
            "rotation": (-70, 0, -90),
            "focal_length": 50,
        },
    }

    for cam_id, cam_params in cameras.items():
        cam_path = f"/Cameras/{cam_id}"
        camera = UsdGeom.Camera.Define(stage, cam_path)

        # 設定攝影機位置
        cam_xform = UsdGeom.Xformable(stage.GetPrimAtPath(cam_path))
        cam_xform.AddTranslateOp().Set(Gf.Vec3d(*cam_params["position"]))

        # 設定旋轉（歐拉角，單位：度）
        rotation = cam_params["rotation"]
        cam_xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))

        # 設定焦距
        camera.CreateFocalLengthAttr(cam_params["focal_length"])

        # 設定感光元件尺寸（35mm全片幅）
        camera.CreateHorizontalApertureAttr(36.0)
        camera.CreateVerticalApertureAttr(24.0)

        print(f"   ✅ 攝影機: {cam_id} - {cam_params['description']}")

# ==================== 添加物理屬性 ====================
def add_physics(stage):
    """為建築物添加物理屬性（碰撞檢測、重力模擬）"""
    print(f"\n📝 添加物理屬性...")

    # 建立物理場景
    physics_scene_path = "/PhysicsScene"
    physics_scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr(9.81)

    print(f"   ✅ 物理場景已建立（重力: 9.81 m/s²）")

    # 為每個樓層添加靜態碰撞器
    for floor_id in FLOOR_HEIGHTS.keys():
        floor_path = f"/Building/{floor_id}/Geometry"
        floor_prim = stage.GetPrimAtPath(floor_path)

        if floor_prim.IsValid():
            # 添加碰撞 API
            UsdPhysics.CollisionAPI.Apply(floor_prim)

            # 設為靜態物體（不受重力影響）
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(floor_prim)
            rigid_body.CreateRigidBodyEnabledAttr(False)

            print(f"   ✅ {floor_id}: 靜態碰撞器已啟用")

# ==================== 建立電梯井 ====================
def create_elevator_shafts(stage):
    """建立貫穿所有樓層的電梯井"""
    print(f"\n📝 建立電梯井...")

    # 電梯井位置（相對於建築中心）
    elevators = [
        {"name": "Elevator_1", "position": (0, 8, 0), "size": (2, 2, 18)},
        {"name": "Elevator_2", "position": (3, 8, 0), "size": (2, 2, 18)},
    ]

    for elev in elevators:
        # 建立電梯井群組
        shaft_path = f"/Building/Elevators/{elev['name']}/Shaft"
        shaft_xform = UsdGeom.Xform.Define(stage, shaft_path)

        # 建立電梯井幾何（長方體）
        shaft_cube_path = f"{shaft_path}/Geometry"
        shaft_cube = UsdGeom.Cube.Define(stage, shaft_cube_path)
        shaft_cube.CreateSizeAttr(1.0)

        # 設定位置和尺寸
        shaft_cube_xform = UsdGeom.Xformable(shaft_cube)
        shaft_cube_xform.AddTranslateOp().Set(Gf.Vec3d(*elev["position"]))
        shaft_cube_xform.AddScaleOp().Set(Gf.Vec3d(*elev["size"]))

        print(f"   ✅ {elev['name']}: 位置 {elev['position']}, 尺寸 {elev['size']}")

# ==================== 建立場景資訊 ====================
def create_scene_metadata(stage):
    """添加場景元數據（建築資訊）"""
    print(f"\n📝 添加場景元數據...")

    # 建立元數據節點
    metadata_path = "/Metadata"
    metadata = stage.DefinePrim(metadata_path)

    # 設定自定義屬性
    metadata.CreateAttribute("building:name", Sdf.ValueTypeNames.String).Set(BUILDING_NAME)
    metadata.CreateAttribute("building:total_floors", Sdf.ValueTypeNames.Int).Set(5)
    metadata.CreateAttribute("building:total_area", Sdf.ValueTypeNames.Float).Set(3100.0)
    metadata.CreateAttribute("building:height", Sdf.ValueTypeNames.Float).Set(18.8)
    metadata.CreateAttribute("building:created_date", Sdf.ValueTypeNames.String).Set("2025-11-23")

    print(f"   ✅ 建築名稱: {BUILDING_NAME}")
    print(f"   ✅ 總樓層數: 5 (B1 + 4F)")
    print(f"   ✅ 總面積: 3,100 m²")

# ==================== 主要組裝流程 ====================
def main():
    """主要執行流程"""

    # 1. 建立場景
    print("📝 步驟 1/8: 建立 USD Stage...")
    stage = create_stage()

    # 建立基本結構
    UsdGeom.Xform.Define(stage, "/Building")
    UsdGeom.Xform.Define(stage, "/Building/Elevators")
    UsdGeom.Xform.Define(stage, "/Environment")
    UsdGeom.Xform.Define(stage, "/Cameras")

    # 2. 載入所有樓層
    print(f"\n📝 步驟 2/8: 載入建築樓層...")
    loaded_floors = []
    for floor_id in ["B1", "1F", "2F", "3F", "4F"]:
        result = load_floor(stage, floor_id)
        if result:
            loaded_floors.append(floor_id)

    if not loaded_floors:
        print(f"\n❌ 錯誤: 沒有載入任何樓層")
        print(f"請先執行 Blender 腳本生成各樓層的 USD 檔案")
        return

    # 3. 建立電梯井
    print(f"\n📝 步驟 3/8: 建立電梯井...")
    create_elevator_shafts(stage)

    # 4. 建立環境照明
    print(f"\n📝 步驟 4/8: 建立環境照明...")
    create_lighting(stage)

    # 5. 建立攝影機
    print(f"\n📝 步驟 5/8: 建立攝影機視角...")
    create_cameras(stage)

    # 6. 添加物理屬性
    print(f"\n📝 步驟 6/8: 添加物理屬性...")
    add_physics(stage)

    # 7. 添加場景元數據
    print(f"\n📝 步驟 7/8: 添加場景元數據...")
    create_scene_metadata(stage)

    # 8. 儲存場景
    print(f"\n📝 步驟 8/8: 儲存場景...")
    stage.Save()

    # 完成
    print(f"\n{'='*70}")
    print(f"  ✅ 完整場景組裝完成！")
    print(f"{'='*70}\n")

    print(f"📊 場景統計:")
    print(f"   - 已載入樓層: {len(loaded_floors)}/5")
    print(f"   - 場景檔案: {OUTPUT_SCENE}")
    print(f"   - 電梯井: 2 座")
    print(f"   - 攝影機: 6 個視角")
    print(f"   - 物理模擬: 已啟用")

    print(f"\n🎬 下一步:")
    print(f"   1. 開啟 Isaac Sim")
    print(f"   2. File → Open → {OUTPUT_SCENE}")
    print(f"   3. 選擇攝影機視角（Viewport → Camera → 選擇攝影機）")
    print(f"   4. 調整材質、光照（Property 面板）")
    print(f"   5. 渲染或匯出影片（Movie Capture）")

    print(f"\n💡 提示:")
    print(f"   - 使用 /Cameras/Overview_Aerial 查看全景")
    print(f"   - 使用 /Cameras/1F_Interior 查看內部空間")
    print(f"   - 按 Play 鍵啟動物理模擬（如有動畫）\n")

if __name__ == "__main__":
    main()
