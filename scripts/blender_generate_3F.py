#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - 3F 三樓建模腳本
家庭支持服務中心

使用方式:
1. 開啟 Blender
2. Scripting 工作區
3. 複製此腳本並執行
4. File → Export → Universal Scene Description (.usdc)
"""

import bpy
import math

# ==================== 工具函數 ====================
def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

def setup_units():
    bpy.context.scene.unit_settings.system = 'METRIC'
    bpy.context.scene.unit_settings.length_unit = 'METERS'

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

def create_wall(name, start, end, height=3.5, thickness=0.2, material=None):
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

def create_room(name, x, y, width, length, height=3.5, floor_material=None, wall_material=None):
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

# ==================== 主要建模函數 ====================
def build_floor_3F():
    """建立 3F 家庭支持服務中心"""

    print("\n" + "="*60)
    print("  開始建立: 3F 家庭支持服務中心")
    print("="*60 + "\n")

    clear_scene()
    setup_units()

    # 建立材質
    print("\n📝 步驟 1: 建立材質...")
    mat_wall = create_material("Material_Wall", (0.95, 0.95, 0.9))
    mat_floor = create_material("Material_Floor", (0.85, 0.85, 0.8))
    mat_counseling = create_material("Material_Counseling", (0.9, 0.9, 0.95))  # 淺紫
    mat_workshop = create_material("Material_Workshop", (1.0, 0.98, 0.9))  # 淺黃
    mat_community = create_material("Material_Community", (0.9, 1.0, 0.9))  # 淺綠
    mat_support = create_material("Material_Support", (0.95, 0.95, 1.0))  # 淺藍

    # 建立主地板
    print("\n📝 步驟 2: 建立主地板...")
    FLOOR_WIDTH = 28.0
    FLOOR_LENGTH = 18.0
    FLOOR_HEIGHT = 3.5

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

    # ==================== 諮商服務區 (150 m²) ====================
    print("\n📝 步驟 4: 建立諮商服務區 (150 m²)...")
    counseling_x = -8.0

    # 個別諮商室 (4間 × 15 m²)
    for i in range(4):
        row = i // 2
        col = i % 2
        x = counseling_x + col * 5
        y = -6 + row * 5
        create_room(f"諮商室{i+1}", x, y, 4, 3.75,
                    floor_material=mat_counseling, wall_material=mat_wall)

    # 家族治療室 (40 m²)
    create_room("家族治療室", counseling_x + 2, 2, 8, 5,
                floor_material=mat_counseling, wall_material=mat_wall)

    # 遊戲治療室 (30 m²)
    create_room("遊戲治療室", counseling_x + 2, 6, 6, 5,
                floor_material=mat_counseling, wall_material=mat_wall)

    # 等候區 (20 m²)
    create_room("等候區", counseling_x - 3, 6, 5, 4,
                floor_material=mat_counseling, wall_material=mat_wall)

    # ==================== 親職教育區 (180 m²) ====================
    print("\n📝 步驟 5: 建立親職教育區 (180 m²)...")
    workshop_x = 5.0

    # 多功能教室 (100 m²)
    create_room("多功能教室", workshop_x, -3, 12.5, 8,
                floor_material=mat_workshop, wall_material=mat_wall)

    # 親子烹飪教室 (50 m²)
    create_room("烹飪教室", workshop_x, 5, 10, 5,
                floor_material=mat_workshop, wall_material=mat_wall)

    # 手作工作坊 (30 m²)
    create_room("手作工坊", workshop_x - 6, 5, 6, 5,
                floor_material=mat_workshop, wall_material=mat_wall)

    # ==================== 社區營造區 (100 m²) ====================
    print("\n📝 步驟 6: 建立社區營造區 (100 m²)...")
    community_x = 8.0
    community_y = -8.0

    # 社區共餐廚房 (60 m²)
    create_room("共餐廚房", community_x, community_y, 10, 6,
                floor_material=mat_community, wall_material=mat_wall)

    # 志工培訓室 (40 m²)
    create_room("志工培訓室", community_x - 5, community_y + 7, 8, 5,
                floor_material=mat_community, wall_material=mat_wall)

    # ==================== 支援空間 (70 m²) ====================
    print("\n📝 步驟 7: 建立支援空間...")
    support_x = -12.0

    # 社工辦公室 (30 m²)
    create_room("社工辦公室", support_x, -2, 6, 5,
                floor_material=mat_support, wall_material=mat_wall)

    # 教材室 (20 m²)
    create_room("教材室", support_x, 3, 5, 4,
                floor_material=mat_support, wall_material=mat_wall)

    # ==================== 添加家具 ====================
    print("\n📝 步驟 8: 添加家具...")
    mat_furniture = create_material("Material_Furniture", (0.7, 0.6, 0.5))

    # 多功能教室 - 桌椅 (50人座)
    for i in range(10):
        row = i // 5
        col = i % 5
        tx = workshop_x - 4 + col * 2
        ty = -6 + row * 2.5

        bpy.ops.mesh.primitive_cube_add(location=(tx, ty, 0.75))
        table = bpy.context.active_object
        table.name = f"Workshop_Table_{i+1}"
        table.scale = (0.8, 0.4, 0.025)

        if table.data.materials:
            table.data.materials[0] = mat_furniture
        else:
            table.data.materials.append(mat_furniture)

    # 烹飪教室 - 中島工作台
    for i in range(2):
        bpy.ops.mesh.primitive_cube_add(location=(workshop_x - 2 + i * 4, 5, 0.9))
        island = bpy.context.active_object
        island.name = f"Kitchen_Island_{i+1}"
        island.scale = (1.5, 0.8, 0.45)

        mat_kitchen = create_material(f"Kitchen_Mat_{i}", (0.9, 0.9, 0.85))
        if island.data.materials:
            island.data.materials[0] = mat_kitchen
        else:
            island.data.materials.append(mat_kitchen)

    # ==================== 照明 ====================
    print("\n📝 步驟 9: 建立照明...")

    rooms_to_light = [
        ("諮商區", counseling_x, 0, 300),
        ("教室區", workshop_x, -3, 500),
        ("廚房區", community_x, community_y, 600),
    ]

    for room_name, lx, ly, intensity in rooms_to_light:
        bpy.ops.object.light_add(type='AREA', location=(lx, ly, FLOOR_HEIGHT - 0.5))
        light = bpy.context.active_object
        light.name = f"{room_name}_Light"
        light.data.energy = intensity
        light.data.size = 4.0

    # ==================== 攝影機 ====================
    print("\n📝 步驟 10: 建立攝影機...")
    bpy.ops.object.camera_add(location=(25, -20, 12))
    camera = bpy.context.active_object
    camera.name = "Camera_Overview"
    camera.rotation_euler = (math.radians(60), 0, math.radians(45))
    bpy.context.scene.camera = camera

    print("\n" + "="*60)
    print("  ✅ 3F 家庭支持服務中心建立完成！")
    print("="*60 + "\n")

    print("📊 統計:")
    print("   - 個別諮商室: 4間")
    print("   - 多功能教室座位: 50人")
    print("   - 烹飪中島: 2組")
    print("\n🎬 下一步: 匯出 USD 並儲存專案\n")

if __name__ == "__main__":
    build_floor_3F()
