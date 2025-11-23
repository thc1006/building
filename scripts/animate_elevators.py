#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - 電梯動畫腳本

功能:
- 建立電梯箱體模型
- 動畫：電梯在樓層間移動 (B1 ↔ 1F ↔ 2F ↔ 3F ↔ 4F)
- 電梯門開關動畫

執行環境: NVIDIA Isaac Sim

使用方式:
1. 先執行 assemble_complete_building.py 建立完整場景
2. 在 Isaac Sim 中開啟 complete_building.usd
3. Window → Script Editor → 載入並執行此腳本
"""

from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf
import omni.usd

# ==================== 配置參數 ====================
SCENE_PATH = "/home/user/building/scenes/complete_building.usd"

# 樓層高度（與 assemble_complete_building.py 一致）
FLOOR_HEIGHTS = {
    "B1": -3.5,
    "1F": 0.0,
    "2F": 4.0,
    "3F": 7.8,
    "4F": 11.3,
}

# 電梯規格
ELEVATOR_SIZE = (1.8, 1.8, 2.4)  # 寬 × 深 × 高 (公尺)
ELEVATOR_POSITIONS = [
    {"id": "Elevator_1", "x": 0, "y": 8},
    {"id": "Elevator_2", "x": 3, "y": 8},
]

# 動畫參數
FPS = 30  # 每秒影格數
ELEVATOR_SPEED = 1.0  # 電梯速度 (公尺/秒)
DOOR_OPEN_TIME = 2.0  # 門開啟時間 (秒)
DOOR_CLOSE_TIME = 2.0  # 門關閉時間 (秒)
STOP_DURATION = 5.0  # 停留時間 (秒)

# ==================== 建立電梯箱體 ====================
def create_elevator_car(stage, elevator_id, position):
    """建立電梯箱體模型"""

    elev_path = f"/Building/Elevators/{elevator_id}"

    print(f"\n📦 建立電梯箱體: {elevator_id}")

    # 建立電梯箱體群組
    car_path = f"{elev_path}/Car"
    car_xform = UsdGeom.Xform.Define(stage, car_path)

    # 設定初始位置（1F）
    car_xform.AddTranslateOp().Set(Gf.Vec3d(position["x"], position["y"], FLOOR_HEIGHTS["1F"] + 1.2))

    # 建立箱體幾何
    # 1. 地板
    floor_path = f"{car_path}/Floor"
    floor = UsdGeom.Cube.Define(stage, floor_path)
    floor.CreateSizeAttr(1.0)

    floor_xform = UsdGeom.Xformable(floor)
    floor_xform.AddScaleOp().Set(Gf.Vec3d(ELEVATOR_SIZE[0], ELEVATOR_SIZE[1], 0.05))
    floor_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))

    # 2. 天花板
    ceiling_path = f"{car_path}/Ceiling"
    ceiling = UsdGeom.Cube.Define(stage, ceiling_path)
    ceiling.CreateSizeAttr(1.0)

    ceiling_xform = UsdGeom.Xformable(ceiling)
    ceiling_xform.AddScaleOp().Set(Gf.Vec3d(ELEVATOR_SIZE[0], ELEVATOR_SIZE[1], 0.05))
    ceiling_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, ELEVATOR_SIZE[2]))

    # 3. 後牆
    back_wall_path = f"{car_path}/BackWall"
    back_wall = UsdGeom.Cube.Define(stage, back_wall_path)
    back_wall.CreateSizeAttr(1.0)

    back_xform = UsdGeom.Xformable(back_wall)
    back_xform.AddScaleOp().Set(Gf.Vec3d(ELEVATOR_SIZE[0], 0.05, ELEVATOR_SIZE[2]))
    back_xform.AddTranslateOp().Set(Gf.Vec3d(0, ELEVATOR_SIZE[1]/2, ELEVATOR_SIZE[2]/2))

    # 4. 左牆
    left_wall_path = f"{car_path}/LeftWall"
    left_wall = UsdGeom.Cube.Define(stage, left_wall_path)
    left_wall.CreateSizeAttr(1.0)

    left_xform = UsdGeom.Xformable(left_wall)
    left_xform.AddScaleOp().Set(Gf.Vec3d(0.05, ELEVATOR_SIZE[1], ELEVATOR_SIZE[2]))
    left_xform.AddTranslateOp().Set(Gf.Vec3d(-ELEVATOR_SIZE[0]/2, 0, ELEVATOR_SIZE[2]/2))

    # 5. 右牆
    right_wall_path = f"{car_path}/RightWall"
    right_wall = UsdGeom.Cube.Define(stage, right_wall_path)
    right_wall.CreateSizeAttr(1.0)

    right_xform = UsdGeom.Xformable(right_wall)
    right_xform.AddScaleOp().Set(Gf.Vec3d(0.05, ELEVATOR_SIZE[1], ELEVATOR_SIZE[2]))
    right_xform.AddTranslateOp().Set(Gf.Vec3d(ELEVATOR_SIZE[0]/2, 0, ELEVATOR_SIZE[2]/2))

    # 6. 左門（可動）
    left_door_path = f"{car_path}/DoorLeft"
    left_door = UsdGeom.Cube.Define(stage, left_door_path)
    left_door.CreateSizeAttr(1.0)

    left_door_xform = UsdGeom.Xformable(left_door)
    left_door_xform.AddScaleOp().Set(Gf.Vec3d(ELEVATOR_SIZE[0]/2, 0.05, ELEVATOR_SIZE[2]))
    left_door_xform.AddTranslateOp().Set(Gf.Vec3d(-ELEVATOR_SIZE[0]/4, -ELEVATOR_SIZE[1]/2, ELEVATOR_SIZE[2]/2))

    # 7. 右門（可動）
    right_door_path = f"{car_path}/DoorRight"
    right_door = UsdGeom.Cube.Define(stage, right_door_path)
    right_door.CreateSizeAttr(1.0)

    right_door_xform = UsdGeom.Xformable(right_door)
    right_door_xform.AddScaleOp().Set(Gf.Vec3d(ELEVATOR_SIZE[0]/2, 0.05, ELEVATOR_SIZE[2]))
    right_door_xform.AddTranslateOp().Set(Gf.Vec3d(ELEVATOR_SIZE[0]/4, -ELEVATOR_SIZE[1]/2, ELEVATOR_SIZE[2]/2))

    # 建立材質（不鏽鋼）
    mat_path = f"/Materials/Elevator_Steel"
    if not stage.GetPrimAtPath(mat_path).IsValid():
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.8, 0.8, 0.85))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.9)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    # 套用材質到所有部件
    material = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    for part in [floor, ceiling, back_wall, left_wall, right_wall, left_door, right_door]:
        UsdShade.MaterialBindingAPI(part).Bind(material)

    print(f"   ✅ 電梯箱體已建立")

    return car_path

# ==================== 動畫：電梯移動 ====================
def animate_elevator_movement(stage, elevator_id, car_path, floor_sequence):
    """
    建立電梯移動動畫

    floor_sequence: 樓層順序列表，例如 ["1F", "2F", "3F", "4F", "3F", "2F", "1F"]
    """

    print(f"\n🎬 建立電梯移動動畫: {elevator_id}")
    print(f"   路線: {' → '.join(floor_sequence)}")

    car_prim = stage.GetPrimAtPath(car_path)
    car_xform = UsdGeom.Xformable(car_prim)

    # 取得位移操作
    translate_op = car_xform.GetOrderedXformOps()[0]

    current_frame = 0

    for i in range(len(floor_sequence)):
        current_floor = floor_sequence[i]
        current_height = FLOOR_HEIGHTS[current_floor] + 1.2  # +1.2m 為箱體中心高度

        # 取得當前位置（X, Y 不變，只改變 Z）
        current_pos = translate_op.Get()

        if i == 0:
            # 第一個關鍵影格：起始位置
            translate_op.Set(Gf.Vec3d(current_pos[0], current_pos[1], current_height), time=current_frame)
            print(f"   影格 {current_frame:4d}: {current_floor} (起點)")

        else:
            # 計算移動距離和時間
            previous_floor = floor_sequence[i-1]
            previous_height = FLOOR_HEIGHTS[previous_floor] + 1.2

            distance = abs(current_height - previous_height)
            travel_time = distance / ELEVATOR_SPEED  # 秒
            travel_frames = int(travel_time * FPS)

            # 移動開始影格（門已關閉）
            move_start_frame = current_frame + int(DOOR_CLOSE_TIME * FPS)

            # 移動結束影格
            move_end_frame = move_start_frame + travel_frames

            # 關鍵影格：移動結束（到達目標樓層）
            translate_op.Set(Gf.Vec3d(current_pos[0], current_pos[1], current_height), time=move_end_frame)

            print(f"   影格 {move_end_frame:4d}: {current_floor} (抵達, {travel_time:.1f}秒)")

            # 更新當前影格（停留時間）
            current_frame = move_end_frame + int(STOP_DURATION * FPS)

    total_duration = current_frame / FPS
    print(f"   總時長: {total_duration:.1f} 秒 ({current_frame} 影格)")

    return current_frame

# ==================== 動畫：電梯門開關 ====================
def animate_elevator_doors(stage, car_path, floor_sequence):
    """建立電梯門開關動畫"""

    print(f"\n🚪 建立電梯門開關動畫")

    left_door_path = f"{car_path}/DoorLeft"
    right_door_path = f"{car_path}/DoorRight"

    left_door = stage.GetPrimAtPath(left_door_path)
    right_door = stage.GetPrimAtPath(right_door_path)

    left_xform = UsdGeom.Xformable(left_door)
    right_xform = UsdGeom.Xformable(right_door)

    # 取得位移操作（第二個 op，因為第一個是 scale）
    left_translate = left_xform.GetOrderedXformOps()[1]
    right_translate = right_xform.GetOrderedXformOps()[1]

    # 門的初始位置
    left_closed = Gf.Vec3d(-ELEVATOR_SIZE[0]/4, -ELEVATOR_SIZE[1]/2, ELEVATOR_SIZE[2]/2)
    right_closed = Gf.Vec3d(ELEVATOR_SIZE[0]/4, -ELEVATOR_SIZE[1]/2, ELEVATOR_SIZE[2]/2)

    # 門的開啟位置（向兩側滑動）
    door_slide = ELEVATOR_SIZE[0] / 2 + 0.1  # 滑動距離
    left_open = Gf.Vec3d(-door_slide, -ELEVATOR_SIZE[1]/2, ELEVATOR_SIZE[2]/2)
    right_open = Gf.Vec3d(door_slide, -ELEVATOR_SIZE[1]/2, ELEVATOR_SIZE[2]/2)

    current_frame = 0

    for i, floor in enumerate(floor_sequence):
        if i == 0:
            # 起始樓層：門開啟
            left_translate.Set(left_open, time=current_frame)
            right_translate.Set(right_open, time=current_frame)
            print(f"   影格 {current_frame:4d}: 門開啟 ({floor})")

        # 關門
        close_start = current_frame
        close_end = current_frame + int(DOOR_CLOSE_TIME * FPS)

        left_translate.Set(left_closed, time=close_end)
        right_translate.Set(right_closed, time=close_end)
        print(f"   影格 {close_end:4d}: 門關閉")

        # 移動中（門保持關閉）
        # ...

        # 計算到達下一樓層的時間
        if i < len(floor_sequence) - 1:
            next_floor = floor_sequence[i+1]
            distance = abs(FLOOR_HEIGHTS[next_floor] - FLOOR_HEIGHTS[floor])
            travel_time = distance / ELEVATOR_SPEED
            travel_frames = int(travel_time * FPS)

            arrival_frame = close_end + travel_frames

            # 開門
            open_start = arrival_frame
            open_end = arrival_frame + int(DOOR_OPEN_TIME * FPS)

            left_translate.Set(left_open, time=open_end)
            right_translate.Set(right_open, time=open_end)
            print(f"   影格 {open_end:4d}: 門開啟 ({next_floor})")

            # 停留時間
            current_frame = open_end + int(STOP_DURATION * FPS)

# ==================== 主程式 ====================
def main():
    """主要執行流程"""

    print("\n" + "="*70)
    print("  電梯動畫生成")
    print("="*70)

    # 開啟場景
    stage = Usd.Stage.Open(SCENE_PATH)

    if not stage:
        print(f"\n❌ 錯誤: 無法開啟場景 {SCENE_PATH}")
        print("請先執行 assemble_complete_building.py")
        return

    print(f"✅ 場景已開啟: {SCENE_PATH}")

    # 設定影格率
    stage.SetTimeCodesPerSecond(FPS)
    print(f"✅ 影格率: {FPS} FPS")

    # 建立電梯箱體
    print(f"\n📝 步驟 1: 建立電梯箱體...")
    for elev in ELEVATOR_POSITIONS:
        car_path = create_elevator_car(stage, elev["id"], elev)

    # 建立動畫：電梯1 (B1 → 1F → 2F → 3F → 4F → 1F)
    print(f"\n📝 步驟 2: 建立電梯1動畫...")
    floor_sequence_1 = ["B1", "1F", "2F", "3F", "4F", "3F", "2F", "1F", "B1"]
    car1_path = f"/Building/Elevators/Elevator_1/Car"
    total_frames_1 = animate_elevator_movement(stage, "Elevator_1", car1_path, floor_sequence_1)
    animate_elevator_doors(stage, car1_path, floor_sequence_1)

    # 建立動畫：電梯2 (1F → 4F → 1F，快速往返)
    print(f"\n📝 步驟 3: 建立電梯2動畫...")
    floor_sequence_2 = ["1F", "4F", "1F"]
    car2_path = f"/Building/Elevators/Elevator_2/Car"
    total_frames_2 = animate_elevator_movement(stage, "Elevator_2", car2_path, floor_sequence_2)
    animate_elevator_doors(stage, car2_path, floor_sequence_2)

    # 設定動畫範圍
    max_frames = max(total_frames_1, total_frames_2)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(max_frames)

    # 儲存場景
    print(f"\n📝 步驟 4: 儲存場景...")
    stage.Save()

    print("\n" + "="*70)
    print("  ✅ 電梯動畫已建立！")
    print("="*70 + "\n")

    print(f"📊 動畫資訊:")
    print(f"   - 總影格數: {max_frames}")
    print(f"   - 總時長: {max_frames / FPS:.1f} 秒")
    print(f"   - 電梯1路線: {' → '.join(floor_sequence_1)}")
    print(f"   - 電梯2路線: {' → '.join(floor_sequence_2)}")

    print(f"\n🎬 播放動畫:")
    print(f"   1. 在 Isaac Sim 中重新載入場景")
    print(f"   2. 按下 Play 按鈕（▶️）")
    print(f"   3. 使用 Timeline 控制播放")
    print(f"   4. 選擇攝影機視角觀看電梯移動\n")

if __name__ == "__main__":
    main()
