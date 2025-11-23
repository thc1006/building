#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - 2F 二樓建模腳本
公共托嬰中心 (0-2歲)

使用方式:
1. 開啟 Blender
2. Scripting 工作區
3. 複製此腳本並執行
4. File → Export → Universal Scene Description (.usdc)
"""

import bpy
import math

# ==================== 工具函數 (與1F相同) ====================
def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    print("✅ 場景已清空")

def setup_units():
    bpy.context.scene.unit_settings.system = 'METRIC'
    bpy.context.scene.unit_settings.length_unit = 'METERS'
    print("✅ 單位設定: 公尺")

def create_material(name, color):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs['Base Color'].default_value = (*color, 1.0)
        bsdf.inputs['Roughness'].default_value = 0.5
    return mat

def create_box(name, location, scale, material=None):
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = scale
    if material:
        if obj.data.materials:
            obj.data.materials[0] = material
        else:
            obj.data.materials.append(material)
    return obj

def create_wall(name, start, end, height=3.8, thickness=0.2, material=None):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.sqrt(dx**2 + dy**2)
    center_x = (start[0] + end[0]) / 2
    center_y = (start[1] + end[1]) / 2
    center_z = height / 2
    angle = math.atan2(dy, dx)

    bpy.ops.mesh.primitive_cube_add(location=(center_x, center_y, center_z))
    wall = bpy.context.active_object
    wall.name = name
    wall.scale = (length / 2, thickness / 2, height / 2)
    wall.rotation_euler = (0, 0, angle)

    if material:
        if wall.data.materials:
            wall.data.materials[0] = material
        else:
            wall.data.materials.append(material)
    return wall

def create_room(name, x, y, width, length, height=3.8, floor_material=None, wall_material=None):
    collection = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(collection)

    floor = create_box(
        f"{name}_Floor",
        location=(x, y, 0.025),
        scale=(width/2, length/2, 0.025),
        material=floor_material
    )
    collection.objects.link(floor)
    bpy.context.scene.collection.objects.unlink(floor)

    half_w = width / 2
    half_l = length / 2

    walls_data = [
        (f"{name}_Wall_S", (x - half_w, y - half_l, 0), (x + half_w, y - half_l, 0)),
        (f"{name}_Wall_N", (x - half_w, y + half_l, 0), (x + half_w, y + half_l, 0)),
        (f"{name}_Wall_W", (x - half_w, y - half_l, 0), (x - half_w, y + half_l, 0)),
        (f"{name}_Wall_E", (x + half_w, y - half_l, 0), (x + half_w, y + half_l, 0)),
    ]

    for wall_name, start, end in walls_data:
        wall = create_wall(wall_name, start, end, height, material=wall_material)
        collection.objects.link(wall)
        bpy.context.scene.collection.objects.unlink(wall)

    print(f"✅ 房間已建立: {name} ({width}m × {length}m)")
    return collection

# ==================== 建立嬰兒床 ====================
def create_crib(name, location):
    """建立嬰兒床模型 (1.2m × 0.6m × 0.9m)"""
    collection = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(collection)

    # 床板
    bpy.ops.mesh.primitive_cube_add(location=(location[0], location[1], 0.4))
    bed = bpy.context.active_object
    bed.name = f"{name}_Mattress"
    bed.scale = (0.6, 0.3, 0.05)

    mat_bed = create_material(f"{name}_Mat", (1.0, 0.95, 0.9))
    if bed.data.materials:
        bed.data.materials[0] = mat_bed
    else:
        bed.data.materials.append(mat_bed)

    collection.objects.link(bed)
    bpy.context.scene.collection.objects.unlink(bed)

    # 床欄（4根柱子+欄杆，簡化為框架）
    mat_wood = create_material(f"{name}_Wood", (0.7, 0.6, 0.5))

    # 四個角柱
    for dx, dy in [(0.6, 0.3), (0.6, -0.3), (-0.6, 0.3), (-0.6, -0.3)]:
        bpy.ops.mesh.primitive_cylinder_add(
            location=(location[0] + dx, location[1] + dy, 0.45),
            radius=0.03, depth=0.9
        )
        post = bpy.context.active_object
        post.name = f"{name}_Post"
        if post.data.materials:
            post.data.materials[0] = mat_wood
        else:
            post.data.materials.append(mat_wood)

        collection.objects.link(post)
        bpy.context.scene.collection.objects.unlink(post)

    return collection

# ==================== 主要建模函數 ====================
def build_floor_2F():
    """建立 2F 公共托嬰中心"""

    print("\n" + "="*60)
    print("  開始建立: 2F 公共托嬰中心 (0-2歲)")
    print("="*60 + "\n")

    clear_scene()
    setup_units()

    # 建立材質
    print("\n📝 步驟 1: 建立材質...")
    mat_wall = create_material("Material_Wall", (0.95, 0.95, 0.9))
    mat_floor = create_material("Material_Floor", (0.85, 0.85, 0.8))
    mat_infant = create_material("Material_Infant", (1.0, 0.95, 0.95))  # 淺粉紅
    mat_toddler = create_material("Material_Toddler", (1.0, 0.98, 0.9))  # 淺黃
    mat_dining = create_material("Material_Dining", (0.95, 1.0, 0.95))  # 淺綠
    mat_outdoor = create_material("Material_Outdoor", (0.9, 0.95, 0.85))  # 草綠
    mat_support = create_material("Material_Support", (0.9, 0.95, 1.0))  # 淺藍

    # 建立主地板
    print("\n📝 步驟 2: 建立主地板...")
    FLOOR_WIDTH = 33.0
    FLOOR_LENGTH = 21.0
    FLOOR_HEIGHT = 3.8

    main_floor = create_box(
        "Main_Floor",
        location=(0, 0, 0.025),
        scale=(FLOOR_WIDTH/2, FLOOR_LENGTH/2, 0.025),
        material=mat_floor
    )

    # 建立外牆
    print("\n📝 步驟 3: 建立外牆...")
    half_w = FLOOR_WIDTH / 2
    half_l = FLOOR_LENGTH / 2

    exterior_walls = [
        ("Exterior_South", (-half_w, -half_l, 0), (half_w, -half_l, 0)),
        ("Exterior_North", (-half_w, half_l, 0), (half_w, half_l, 0)),
        ("Exterior_West", (-half_w, -half_l, 0), (-half_w, half_l, 0)),
        ("Exterior_East", (half_w, -half_l, 0), (half_w, half_l, 0)),
    ]

    for name, start, end in exterior_walls:
        create_wall(name, start, end, FLOOR_HEIGHT, thickness=0.30, material=mat_wall)

    # ==================== 嬰兒室 0-1歲 (180 m²) ====================
    print("\n📝 步驟 4: 建立嬰兒室 0-1歲 (180 m²)...")
    infant_x = -10.0

    # 嬰兒遊戲區 (80 m²)
    create_room("嬰兒_遊戲區", infant_x, -5, 10, 8,
                floor_material=mat_infant, wall_material=mat_wall)

    # 嬰兒午睡室 (60 m²) - 15張嬰兒床
    create_room("嬰兒_午睡室", infant_x, 4, 8, 7.5,
                floor_material=mat_infant, wall_material=mat_wall)

    # 在午睡室中添加嬰兒床
    print("   - 添加嬰兒床 (15張)...")
    for i in range(15):
        row = i // 5
        col = i % 5
        x = infant_x - 3 + col * 1.5
        y = 1 + row * 2.0
        create_crib(f"Crib_{i+1}", (x, y, 0))

    # 調乳室 (20 m²)
    create_room("嬰兒_調乳室", infant_x + 6, -5, 5, 4,
                floor_material=mat_infant, wall_material=mat_wall)

    # 尿布更換區 (20 m²)
    create_room("嬰兒_更換區", infant_x + 6, 0, 5, 4,
                floor_material=mat_infant, wall_material=mat_wall)

    # ==================== 幼兒室 1-2歲 (250 m²) ====================
    print("\n📝 步驟 5: 建立幼兒室 1-2歲 (250 m²)...")
    toddler_x = 6.0

    # 幼兒遊戲區 (120 m²)
    create_room("幼兒_遊戲區", toddler_x, -3, 12, 10,
                floor_material=mat_toddler, wall_material=mat_wall)

    # 幼兒午睡室 (80 m²)
    create_room("幼兒_午睡室", toddler_x, 6, 10, 8,
                floor_material=mat_toddler, wall_material=mat_wall)

    # 閱讀角 (30 m²)
    create_room("幼兒_閱讀角", toddler_x - 7, 6, 6, 5,
                floor_material=mat_toddler, wall_material=mat_wall)

    # 感覺統合區 (20 m²)
    create_room("幼兒_感統區", toddler_x + 8, 6, 5, 4,
                floor_material=mat_toddler, wall_material=mat_wall)

    # ==================== 共享設施 (170 m²) ====================
    print("\n📝 步驟 6: 建立共享設施...")
    shared_x = 12.0

    # 幼兒餐廳 (60 m²)
    create_room("共用_餐廳", shared_x, -8, 10, 6,
                floor_material=mat_dining, wall_material=mat_wall)

    # 備餐區 (25 m²)
    create_room("共用_備餐區", shared_x + 7, -8, 5, 5,
                floor_material=mat_dining, wall_material=mat_wall)

    # 戶外遊戲區 (85 m²) - 陽台改造
    outdoor_x = 10.0
    outdoor_y = 8.0

    # 戶外地板（使用不同顏色）
    bpy.ops.mesh.primitive_plane_add(location=(outdoor_x, outdoor_y, 0.05))
    outdoor_floor = bpy.context.active_object
    outdoor_floor.name = "Outdoor_Play_Area"
    outdoor_floor.scale = (8.5/2, 10/2, 1)

    if outdoor_floor.data.materials:
        outdoor_floor.data.materials[0] = mat_outdoor
    else:
        outdoor_floor.data.materials.append(mat_outdoor)

    # 戶外圍欄（透明壓克力，簡化為邊框）
    fence_height = 1.5
    mat_fence = create_material("Material_Fence", (0.8, 0.9, 1.0))

    fence_walls = [
        ("Fence_N", (outdoor_x - 4.25, outdoor_y + 5, 0), (outdoor_x + 4.25, outdoor_y + 5, 0)),
        ("Fence_S", (outdoor_x - 4.25, outdoor_y - 5, 0), (outdoor_x + 4.25, outdoor_y - 5, 0)),
        ("Fence_E", (outdoor_x + 4.25, outdoor_y - 5, 0), (outdoor_x + 4.25, outdoor_y + 5, 0)),
    ]

    for name, start, end in fence_walls:
        create_wall(name, start, end, fence_height, thickness=0.05, material=mat_fence)

    # ==================== 支援空間 (100 m²) ====================
    print("\n📝 步驟 7: 建立支援空間...")
    support_x = -14.0

    # 行政辦公室 (25 m²)
    create_room("支援_辦公室", support_x, 0, 5, 5,
                floor_material=mat_support, wall_material=mat_wall)

    # 保健室 (15 m²)
    create_room("支援_保健室", support_x, 6, 5, 3,
                floor_material=mat_support, wall_material=mat_wall)

    # 親子廁所 (30 m²)
    create_room("支援_廁所", support_x, -6, 6, 5,
                floor_material=mat_support, wall_material=mat_wall)

    # ==================== 添加簡單家具 ====================
    print("\n📝 步驟 8: 添加家具...")

    # 在餐廳添加桌子
    mat_table = create_material("Material_Table", (0.8, 0.7, 0.6))
    for i in range(10):
        row = i // 5
        col = i % 5
        tx = shared_x - 3 + col * 1.8
        ty = -8 + row * 2.5

        bpy.ops.mesh.primitive_cube_add(location=(tx, ty, 0.5))
        table = bpy.context.active_object
        table.name = f"Table_{i+1}"
        table.scale = (0.4, 0.3, 0.025)

        if table.data.materials:
            table.data.materials[0] = mat_table
        else:
            table.data.materials.append(mat_table)

    # ==================== 照明 ====================
    print("\n📝 步驟 9: 建立照明...")

    # 柔和區域光（適合嬰幼兒）
    rooms_to_light = [
        ("嬰兒區", infant_x, -5, 400),
        ("幼兒區", toddler_x, -3, 400),
        ("餐廳", shared_x, -8, 350),
        ("戶外", outdoor_x, outdoor_y, 500),
    ]

    for room_name, lx, ly, intensity in rooms_to_light:
        bpy.ops.object.light_add(type='AREA', location=(lx, ly, FLOOR_HEIGHT - 0.5))
        light = bpy.context.active_object
        light.name = f"{room_name}_Light"
        light.data.energy = intensity
        light.data.size = 4.0
        light.data.color = (1.0, 0.98, 0.95)  # 溫暖色溫

    # ==================== 攝影機 ====================
    print("\n📝 步驟 10: 建立攝影機...")
    bpy.ops.object.camera_add(location=(30, -25, 15))
    camera = bpy.context.active_object
    camera.name = "Camera_Overview"
    camera.rotation_euler = (math.radians(60), 0, math.radians(45))
    bpy.context.scene.camera = camera

    print("\n" + "="*60)
    print("  ✅ 2F 公共托嬰中心建立完成！")
    print("="*60 + "\n")

    print("📊 統計:")
    print("   - 嬰兒床: 15張")
    print("   - 幼兒桌: 10張")
    print("   - 戶外遊戲區: 85 m²")
    print("\n🎬 下一步:")
    print("   1. 匯出 USD: File → Export → USD (.usdc)")
    print("   2. 儲存檔案: File → Save As → floor_2F.blend\n")

if __name__ == "__main__":
    build_floor_2F()
