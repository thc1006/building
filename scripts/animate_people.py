#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - 人員移動動畫腳本

功能:
- 建立簡化的人物模型（長者、幼兒、青少年、工作人員）
- 動畫：人員在建築物內移動
- 模擬各樓層的使用情境

執行環境: NVIDIA Isaac Sim

使用方式:
1. 先執行 assemble_complete_building.py 建立完整場景
2. 在 Isaac Sim 中開啟 complete_building.usd
3. Window → Script Editor → 載入並執行此腳本

進階: 使用 NVIDIA Character Generator 建立更真實的人物模型
"""

from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf
import omni.usd
import math

# ==================== 配置參數 ====================
SCENE_PATH = "/home/user/building/scenes/complete_building.usd"

# 動畫參數
FPS = 30
WALK_SPEED = 1.2  # 正常步行速度 (m/s)
ELDERLY_WALK_SPEED = 0.6  # 長者步行速度 (m/s)
CHILD_RUN_SPEED = 2.0  # 兒童奔跑速度 (m/s)

# 人物尺寸
PERSON_HEIGHTS = {
    "elderly": 1.65,  # 長者
    "adult": 1.70,  # 成人
    "child": 1.0,  # 幼兒
    "youth": 1.60,  # 青少年
}

PERSON_RADIUS = 0.25  # 人物半徑（圓柱體）

# 人物顏色（簡化表示）
PERSON_COLORS = {
    "elderly": (0.7, 0.7, 0.8),  # 灰藍色
    "adult": (0.3, 0.5, 0.7),  # 藍色（工作人員）
    "child": (1.0, 0.8, 0.6),  # 淺橘色
    "youth": (0.8, 0.3, 0.4),  # 紫紅色
}

# ==================== 建立簡化人物模型 ====================
def create_person(stage, person_id, person_type, position):
    """
    建立簡化的人物模型（圓柱體表示）

    person_type: "elderly", "adult", "child", "youth"
    position: (x, y, z) 初始位置
    """

    height = PERSON_HEIGHTS[person_type]
    color = PERSON_COLORS[person_type]

    print(f"👤 建立人物: {person_id} (類型: {person_type})")

    # 建立人物群組
    person_path = f"/Building/People/{person_id}"
    person_xform = UsdGeom.Xform.Define(stage, person_path)

    # 身體（圓柱體）
    body_path = f"{person_path}/Body"
    body = UsdGeom.Cylinder.Define(stage, body_path)
    body.CreateHeightAttr(height)
    body.CreateRadiusAttr(PERSON_RADIUS)

    # 設定位置（圓柱體中心在地面上方 height/2）
    body_xform = UsdGeom.Xformable(body)
    body_xform.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], position[2] + height/2))

    # 頭部（球體）
    head_path = f"{person_path}/Head"
    head = UsdGeom.Sphere.Define(stage, head_path)
    head.CreateRadiusAttr(0.15)

    head_xform = UsdGeom.Xformable(head)
    head_xform.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], position[2] + height + 0.15))

    # 建立材質
    mat_path = f"/Materials/Person_{person_type}"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    material = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI(body).Bind(material)

    # 頭部使用淺色
    head_mat_path = f"/Materials/Person_Head"
    if not stage.GetPrimAtPath(head_mat_path).IsValid():
        head_material = UsdShade.Material.Define(stage, head_mat_path)
        head_shader = UsdShade.Shader.Define(stage, f"{head_mat_path}/Shader")
        head_shader.CreateIdAttr("UsdPreviewSurface")
        head_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.95, 0.85, 0.75))
        head_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        head_material.CreateSurfaceOutput().ConnectToSource(head_shader.ConnectableAPI(), "surface")

    head_material = UsdShade.Material(stage.GetPrimAtPath(head_mat_path))
    UsdShade.MaterialBindingAPI(head).Bind(head_material)

    print(f"   ✅ 人物已建立: 高度 {height}m")

    return person_path, body_path, head_path

# ==================== 動畫：直線移動 ====================
def animate_walk(stage, body_path, head_path, waypoints, speed=WALK_SPEED):
    """
    建立行走動畫（沿路徑點移動）

    waypoints: 路徑點列表 [(x1, y1, z1), (x2, y2, z2), ...]
    speed: 移動速度 (m/s)
    """

    body_prim = stage.GetPrimAtPath(body_path)
    head_prim = stage.GetPrimAtPath(head_path)

    body_xform = UsdGeom.Xformable(body_prim)
    head_xform = UsdGeom.Xformable(head_prim)

    body_translate = body_xform.GetOrderedXformOps()[0]
    head_translate = head_xform.GetOrderedXformOps()[0]

    current_frame = 0

    for i in range(len(waypoints)):
        current_point = waypoints[i]

        # 計算身體和頭部的位置
        person_type = "adult"  # 預設
        height = PERSON_HEIGHTS[person_type]

        body_pos = Gf.Vec3d(current_point[0], current_point[1], current_point[2] + height/2)
        head_pos = Gf.Vec3d(current_point[0], current_point[1], current_point[2] + height + 0.15)

        # 設定關鍵影格
        body_translate.Set(body_pos, time=current_frame)
        head_translate.Set(head_pos, time=current_frame)

        # 計算到下一個點的時間
        if i < len(waypoints) - 1:
            next_point = waypoints[i + 1]
            distance = math.sqrt(
                (next_point[0] - current_point[0])**2 +
                (next_point[1] - current_point[1])**2 +
                (next_point[2] - current_point[2])**2
            )
            travel_time = distance / speed
            travel_frames = int(travel_time * FPS)
            current_frame += travel_frames

    print(f"   ✅ 行走動畫: {len(waypoints)} 個路徑點, {current_frame / FPS:.1f} 秒")

    return current_frame

# ==================== 場景：1F 長者散步 ====================
def scene_1F_elderly_walk(stage):
    """1F 長者在庭園散步"""

    print("\n🎬 場景 1: 1F 長者庭園散步")

    # 建立2位長者
    elderly1_path, body1, head1 = create_person(stage, "Elderly_1", "elderly", (10, -5, 0))
    elderly2_path, body2, head2 = create_person(stage, "Elderly_2", "elderly", (12, -5, 0))

    # 長者1路徑：從餐廳 → 庭園 → 休息室
    waypoints1 = [
        (10, -5, 0),  # 起點：餐廳
        (12, 0, 0),   # 庭園入口
        (14, 2, 0),   # 庭園中央
        (12, 4, 0),   # 庭園出口
        (8, 6, 0),    # 休息室
    ]

    animate_walk(stage, body1, head1, waypoints1, speed=ELDERLY_WALK_SPEED)

    # 長者2路徑：從休息室 → 活動室
    waypoints2 = [
        (12, -5, 0),  # 起點
        (5, -3, 0),   # 活動室入口
        (3, -1, 0),   # 活動室內
    ]

    animate_walk(stage, body2, head2, waypoints2, speed=ELDERLY_WALK_SPEED)

# ==================== 場景：2F 幼兒遊戲 ====================
def scene_2F_children_play(stage):
    """2F 幼兒在遊戲區活動"""

    print("\n🎬 場景 2: 2F 幼兒遊戲區")

    # 建立3位幼兒
    child1_path, body1, head1 = create_person(stage, "Child_1", "child", (6, -3, 4))
    child2_path, body2, head2 = create_person(stage, "Child_2", "child", (8, -3, 4))
    child3_path, body3, head3 = create_person(stage, "Child_3", "child", (10, -3, 4))

    # 幼兒1: 在遊戲區跑動（圓形路徑）
    waypoints1 = []
    center_x, center_y = 6, -3
    radius = 3
    for angle in range(0, 360, 30):
        rad = math.radians(angle)
        x = center_x + radius * math.cos(rad)
        y = center_y + radius * math.sin(rad)
        waypoints1.append((x, y, 4))

    animate_walk(stage, body1, head1, waypoints1, speed=CHILD_RUN_SPEED)

    # 幼兒2和3: 從室內 → 戶外遊戲區
    waypoints2 = [
        (8, -3, 4),   # 室內遊戲區
        (10, 2, 4),   # 走廊
        (10, 6, 4),   # 戶外遊戲區
    ]

    waypoints3 = [
        (10, -3, 4),
        (10, 0, 4),
        (12, 4, 4),
        (12, 8, 4),   # 戶外遊戲區
    ]

    animate_walk(stage, body2, head2, waypoints2, speed=WALK_SPEED)
    animate_walk(stage, body3, head3, waypoints3, speed=WALK_SPEED)

# ==================== 場景：4F 青少年打籃球 ====================
def scene_4F_youth_basketball(stage):
    """4F 青少年在籃球場移動"""

    print("\n🎬 場景 3: 4F 青少年籃球場")

    # 建立4位青少年（模擬籃球移動）
    positions = [
        (-8, -5, 11.3),
        (-2, -5, 11.3),
        (-8, -1, 11.3),
        (-2, -1, 11.3),
    ]

    for i, pos in enumerate(positions):
        person_path, body, head = create_person(stage, f"Youth_{i+1}", "youth", pos)

        # 簡單的來回移動
        waypoints = [
            pos,
            (pos[0] + 3, pos[1] + 2, pos[2]),
            (pos[0], pos[1] + 4, pos[2]),
            (pos[0] - 3, pos[1] + 2, pos[2]),
            pos,
        ]

        animate_walk(stage, body, head, waypoints, speed=CHILD_RUN_SPEED)

# ==================== 場景：工作人員巡視 ====================
def scene_staff_patrol(stage):
    """工作人員巡視各樓層"""

    print("\n🎬 場景 4: 工作人員巡視")

    # 護理師在1F巡視
    staff1_path, body1, head1 = create_person(stage, "Nurse_1", "adult", (-15, 0, 0))

    waypoints_nurse = [
        (-15, 0, 0),   # 護理站
        (-10, -5, 0),  # 失智專區
        (-10, 5, 0),   # 感官刺激室
        (5, -3, 0),    # 活動室
        (12, 0, 0),    # 餐廳
        (-15, 0, 0),   # 回護理站
    ]

    animate_walk(stage, body1, head1, waypoints_nurse, speed=WALK_SPEED)

    # 教保員在2F
    staff2_path, body2, head2 = create_person(stage, "Teacher_1", "adult", (-14, 0, 4))

    waypoints_teacher = [
        (-14, 0, 4),   # 辦公室
        (-10, -5, 4),  # 嬰兒室
        (6, -3, 4),    # 幼兒遊戲區
        (12, -8, 4),   # 餐廳
        (-14, 0, 4),   # 回辦公室
    ]

    animate_walk(stage, body2, head2, waypoints_teacher, speed=WALK_SPEED)

# ==================== 主程式 ====================
def main():
    """主要執行流程"""

    print("\n" + "="*70)
    print("  人員移動動畫生成")
    print("="*70)

    # 開啟場景
    stage = Usd.Stage.Open(SCENE_PATH)

    if not stage:
        print(f"\n❌ 錯誤: 無法開啟場景 {SCENE_PATH}")
        return

    print(f"✅ 場景已開啟: {SCENE_PATH}")

    # 設定影格率
    stage.SetTimeCodesPerSecond(FPS)

    # 建立 People 群組
    UsdGeom.Xform.Define(stage, "/Building/People")

    # 建立各場景
    print(f"\n{'='*70}")
    print("建立場景動畫...")

    scene_1F_elderly_walk(stage)
    scene_2F_children_play(stage)
    scene_4F_youth_basketball(stage)
    scene_staff_patrol(stage)

    # 設定動畫範圍（30秒循環）
    total_frames = int(30 * FPS)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(total_frames)

    # 儲存場景
    print(f"\n📝 儲存場景...")
    stage.Save()

    print("\n" + "="*70)
    print("  ✅ 人員移動動畫已建立！")
    print("="*70 + "\n")

    print(f"📊 動畫資訊:")
    print(f"   - 總人數: 12 人")
    print(f"   - 長者: 2 人")
    print(f"   - 幼兒: 3 人")
    print(f"   - 青少年: 4 人")
    print(f"   - 工作人員: 2 人")
    print(f"   - 動畫時長: 30 秒")
    print(f"   - 總影格數: {total_frames}")

    print(f"\n🎬 播放動畫:")
    print(f"   1. 在 Isaac Sim 中重新載入場景")
    print(f"   2. 按下 Play 按鈕（▶️）")
    print(f"   3. 切換攝影機視角觀看各樓層活動")

    print(f"\n💡 進階提示:")
    print(f"   - 使用 NVIDIA Character Generator 建立真實人物模型")
    print(f"   - 使用 Isaac Sim 的 Crowd Simulation 進行大規模人群模擬")
    print(f"   - 添加骨骼動畫使人物行走更自然\n")

if __name__ == "__main__":
    main()
