#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - 門開關動畫腳本

功能:
- 為建築物的主要門添加自動開關動畫
- 支援滑動門、推拉門、旋轉門
- 模擬人員進出時的門自動感應

執行環境: NVIDIA Isaac Sim

使用方式:
1. 先執行 assemble_complete_building.py 建立完整場景
2. 在 Isaac Sim 中開啟 complete_building.usd
3. Window → Script Editor → 載入並執行此腳本
"""

from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf
import omni.usd
import math

# ==================== 配置參數 ====================
SCENE_PATH = "/home/user/building/scenes/complete_building.usd"

# 動畫參數
FPS = 30
DOOR_WIDTH = 1.2  # 門寬 (公尺)
DOOR_HEIGHT = 2.2  # 門高 (公尺)
DOOR_THICKNESS = 0.05  # 門厚度 (公尺)

# 門的位置配置
DOORS_CONFIG = [
    {
        "id": "1F_Main_Entrance",
        "floor": "1F",
        "type": "sliding",  # 滑動門
        "position": (0, -11.5, 1.1),  # X, Y, Z
        "width": 2.4,  # 雙開門
        "description": "1F 主入口（自動滑動門）",
    },
    {
        "id": "2F_Nursery_Entrance",
        "floor": "2F",
        "type": "push",  # 推門
        "position": (-10, -10.5, 5.1),
        "width": 1.2,
        "description": "2F 托嬰中心入口",
    },
    {
        "id": "3F_Counseling_Door",
        "floor": "3F",
        "type": "sliding",
        "position": (-8, -9, 8.9),
        "width": 1.0,
        "description": "3F 諮商室門",
    },
    {
        "id": "4F_Basketball_Door",
        "floor": "4F",
        "type": "double_push",  # 雙開推門
        "position": (-5, -8, 13.3),
        "width": 1.6,
        "description": "4F 籃球場入口",
    },
]

# ==================== 建立滑動門模型 ====================
def create_sliding_door(stage, door_config):
    """建立滑動門（左右分開）"""

    door_id = door_config["id"]
    position = door_config["position"]
    width = door_config["width"]

    print(f"\n🚪 建立滑動門: {door_id}")

    door_path = f"/Building/Doors/{door_id}"
    door_xform = UsdGeom.Xform.Define(stage, door_path)

    # 左門
    left_door_path = f"{door_path}/Left"
    left_door = UsdGeom.Cube.Define(stage, left_door_path)
    left_door.CreateSizeAttr(1.0)

    left_xform = UsdGeom.Xformable(left_door)
    left_xform.AddScaleOp().Set(Gf.Vec3d(width/2, DOOR_THICKNESS, DOOR_HEIGHT))
    left_xform.AddTranslateOp().Set(Gf.Vec3d(position[0] - width/4, position[1], position[2]))

    # 右門
    right_door_path = f"{door_path}/Right"
    right_door = UsdGeom.Cube.Define(stage, right_door_path)
    right_door.CreateSizeAttr(1.0)

    right_xform = UsdGeom.Xformable(right_door)
    right_xform.AddScaleOp().Set(Gf.Vec3d(width/2, DOOR_THICKNESS, DOOR_HEIGHT))
    right_xform.AddTranslateOp().Set(Gf.Vec3d(position[0] + width/4, position[1], position[2]))

    # 建立玻璃材質
    create_glass_material(stage, left_door)
    create_glass_material(stage, right_door)

    print(f"   ✅ 滑動門已建立（寬度: {width}m）")

    return door_path, left_door_path, right_door_path

# ==================== 建立推門模型 ====================
def create_push_door(stage, door_config):
    """建立推門（單開，旋轉開啟）"""

    door_id = door_config["id"]
    position = door_config["position"]
    width = door_config["width"]

    print(f"\n🚪 建立推門: {door_id}")

    door_path = f"/Building/Doors/{door_id}"
    door_xform = UsdGeom.Xform.Define(stage, door_path)

    # 門扇
    panel_path = f"{door_path}/Panel"
    panel = UsdGeom.Cube.Define(stage, panel_path)
    panel.CreateSizeAttr(1.0)

    panel_xform = UsdGeom.Xformable(panel)
    panel_xform.AddScaleOp().Set(Gf.Vec3d(width, DOOR_THICKNESS, DOOR_HEIGHT))
    panel_xform.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], position[2]))

    # 建立木質材質
    create_wood_material(stage, panel)

    print(f"   ✅ 推門已建立（寬度: {width}m）")

    return door_path, panel_path

# ==================== 建立雙開推門 ====================
def create_double_push_door(stage, door_config):
    """建立雙開推門"""

    door_id = door_config["id"]
    position = door_config["position"]
    width = door_config["width"]

    print(f"\n🚪 建立雙開推門: {door_id}")

    door_path = f"/Building/Doors/{door_id}"
    door_xform = UsdGeom.Xform.Define(stage, door_path)

    # 左門
    left_panel_path = f"{door_path}/LeftPanel"
    left_panel = UsdGeom.Cube.Define(stage, left_panel_path)
    left_panel.CreateSizeAttr(1.0)

    left_xform = UsdGeom.Xformable(left_panel)
    left_xform.AddScaleOp().Set(Gf.Vec3d(width/2, DOOR_THICKNESS, DOOR_HEIGHT))
    left_xform.AddTranslateOp().Set(Gf.Vec3d(position[0] - width/4, position[1], position[2]))

    # 右門
    right_panel_path = f"{door_path}/RightPanel"
    right_panel = UsdGeom.Cube.Define(stage, right_panel_path)
    right_panel.CreateSizeAttr(1.0)

    right_xform = UsdGeom.Xformable(right_panel)
    right_xform.AddScaleOp().Set(Gf.Vec3d(width/2, DOOR_THICKNESS, DOOR_HEIGHT))
    right_xform.AddTranslateOp().Set(Gf.Vec3d(position[0] + width/4, position[1], position[2]))

    create_wood_material(stage, left_panel)
    create_wood_material(stage, right_panel)

    print(f"   ✅ 雙開推門已建立（寬度: {width}m）")

    return door_path, left_panel_path, right_panel_path

# ==================== 材質：玻璃 ====================
def create_glass_material(stage, prim):
    """建立玻璃材質（透明）"""

    mat_path = "/Materials/Glass"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.8, 0.9, 1.0))
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.3)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.1)
        shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    material = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI(prim).Bind(material)

# ==================== 材質：木質 ====================
def create_wood_material(stage, prim):
    """建立木質材質"""

    mat_path = "/Materials/Wood"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.6, 0.4, 0.2))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    material = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI(prim).Bind(material)

# ==================== 動畫：滑動門開關 ====================
def animate_sliding_door(stage, left_path, right_path, open_time, close_time, cycle_duration):
    """滑動門開關動畫"""

    left_prim = stage.GetPrimAtPath(left_path)
    right_prim = stage.GetPrimAtPath(right_path)

    left_xform = UsdGeom.Xformable(left_prim)
    right_xform = UsdGeom.Xformable(right_prim)

    left_translate = left_xform.GetOrderedXformOps()[1]
    right_translate = right_xform.GetOrderedXformOps()[1]

    # 取得初始位置
    left_pos = left_translate.Get()
    right_pos = right_translate.Get()

    # 計算開啟位置（向兩側滑動）
    slide_distance = 1.2  # 滑動距離
    left_open = Gf.Vec3d(left_pos[0] - slide_distance, left_pos[1], left_pos[2])
    right_open = Gf.Vec3d(right_pos[0] + slide_distance, right_pos[1], right_pos[2])

    # 關鍵影格
    frame_open_start = 0
    frame_open_end = int(open_time * FPS)
    frame_close_start = int((cycle_duration - close_time) * FPS)
    frame_close_end = int(cycle_duration * FPS)

    # 動畫序列
    # 0s: 門關閉
    left_translate.Set(left_pos, time=frame_open_start)
    right_translate.Set(right_pos, time=frame_open_start)

    # open_time: 門完全開啟
    left_translate.Set(left_open, time=frame_open_end)
    right_translate.Set(right_open, time=frame_open_end)

    # 保持開啟狀態
    left_translate.Set(left_open, time=frame_close_start)
    right_translate.Set(right_open, time=frame_close_start)

    # cycle_duration: 門關閉
    left_translate.Set(left_pos, time=frame_close_end)
    right_translate.Set(right_pos, time=frame_close_end)

    print(f"   ✅ 滑動門動畫: 開啟 {open_time}s, 關閉 {close_time}s, 週期 {cycle_duration}s")

# ==================== 動畫：推門開關 ====================
def animate_push_door(stage, panel_path, open_time, close_time, cycle_duration, open_angle=90):
    """推門旋轉動畫"""

    panel_prim = stage.GetPrimAtPath(panel_path)
    panel_xform = UsdGeom.Xformable(panel_prim)

    # 添加旋轉操作（繞 Z 軸）
    rotate_op = panel_xform.AddRotateZOp()

    # 關鍵影格
    frame_open_start = 0
    frame_open_end = int(open_time * FPS)
    frame_close_start = int((cycle_duration - close_time) * FPS)
    frame_close_end = int(cycle_duration * FPS)

    # 動畫序列
    rotate_op.Set(0, time=frame_open_start)  # 關閉
    rotate_op.Set(open_angle, time=frame_open_end)  # 開啟
    rotate_op.Set(open_angle, time=frame_close_start)  # 保持開啟
    rotate_op.Set(0, time=frame_close_end)  # 關閉

    print(f"   ✅ 推門動畫: 旋轉 {open_angle}°, 週期 {cycle_duration}s")

# ==================== 主程式 ====================
def main():
    """主要執行流程"""

    print("\n" + "="*70)
    print("  門開關動畫生成")
    print("="*70)

    # 開啟場景
    stage = Usd.Stage.Open(SCENE_PATH)

    if not stage:
        print(f"\n❌ 錯誤: 無法開啟場景 {SCENE_PATH}")
        return

    print(f"✅ 場景已開啟: {SCENE_PATH}")

    # 設定影格率
    stage.SetTimeCodesPerSecond(FPS)

    # 建立 Doors 群組
    UsdGeom.Xform.Define(stage, "/Building/Doors")

    # 處理每個門
    for i, door_config in enumerate(DOORS_CONFIG):
        print(f"\n{'='*70}")
        print(f"處理門 {i+1}/{len(DOORS_CONFIG)}: {door_config['description']}")

        if door_config["type"] == "sliding":
            # 滑動門
            door_path, left_path, right_path = create_sliding_door(stage, door_config)
            animate_sliding_door(stage, left_path, right_path,
                               open_time=2.0, close_time=2.0, cycle_duration=10.0)

        elif door_config["type"] == "push":
            # 單開推門
            door_path, panel_path = create_push_door(stage, door_config)
            animate_push_door(stage, panel_path,
                            open_time=1.5, close_time=1.5, cycle_duration=8.0, open_angle=90)

        elif door_config["type"] == "double_push":
            # 雙開推門
            door_path, left_path, right_path = create_double_push_door(stage, door_config)
            # 左門向左開，右門向右開
            animate_push_door(stage, left_path,
                            open_time=1.5, close_time=1.5, cycle_duration=8.0, open_angle=-90)
            animate_push_door(stage, right_path,
                            open_time=1.5, close_time=1.5, cycle_duration=8.0, open_angle=90)

    # 設定動畫範圍
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(int(10.0 * FPS))  # 10秒循環

    # 儲存場景
    print(f"\n📝 儲存場景...")
    stage.Save()

    print("\n" + "="*70)
    print("  ✅ 門開關動畫已建立！")
    print("="*70 + "\n")

    print(f"📊 動畫資訊:")
    print(f"   - 門的數量: {len(DOORS_CONFIG)}")
    print(f"   - 動畫週期: 10 秒")
    print(f"   - 總影格數: {int(10.0 * FPS)}")

    print(f"\n🎬 播放動畫:")
    print(f"   1. 在 Isaac Sim 中重新載入場景")
    print(f"   2. 按下 Play 按鈕（▶️）")
    print(f"   3. 觀察各樓層的門自動開關\n")

if __name__ == "__main__":
    main()
