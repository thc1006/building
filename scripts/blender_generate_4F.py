#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - 4F 四樓建模腳本
青少年活動中心

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

def create_wall(name, start, end, height=4.0, thickness=0.2, material=None):
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

def create_room(name, x, y, width, length, height=4.0, floor_material=None, wall_material=None):
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
def build_floor_4F():
    """建立 4F 青少年活動中心"""

    print("\n" + "="*60)
    print("  開始建立: 4F 青少年活動中心")
    print("="*60 + "\n")

    clear_scene()
    setup_units()

    # 建立材質
    print("\n📝 步驟 1: 建立材質...")
    mat_wall = create_material("Material_Wall", (0.95, 0.95, 0.9))
    mat_floor = create_material("Material_Floor", (0.85, 0.85, 0.8))
    mat_sports = create_material("Material_Sports", (1.0, 0.95, 0.9))  # 淺橘
    mat_learning = create_material("Material_Learning", (0.9, 1.0, 1.0))  # 淺青
    mat_social = create_material("Material_Social", (1.0, 0.9, 0.95))  # 淺粉
    mat_basketball = create_material("Material_Basketball", (0.85, 0.7, 0.5))  # 木質地板色

    # 建立主地板
    print("\n📝 步驟 2: 建立主地板...")
    FLOOR_WIDTH = 28.0
    FLOOR_LENGTH = 18.0
    FLOOR_HEIGHT = 4.0  # 運動區需要較高

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

    # ==================== 運動休閒區 (220 m²) ====================
    print("\n📝 步驟 4: 建立運動休閒區 (220 m²)...")
    sports_x = -5.0

    # 室內籃球場 (150 m²) - 15m × 10m, 天花板高 6m
    basketball_height = 6.0
    basketball_room = create_room("籃球場", sports_x, -3, 15, 10, height=basketball_height,
                                  floor_material=mat_basketball, wall_material=mat_wall)

    # 添加籃框（簡化為圓環）
    for side in [-1, 1]:
        bpy.ops.mesh.primitive_torus_add(
            location=(sports_x + side * 6, -3, 3.05),
            major_radius=0.23,
            minor_radius=0.02,
            rotation=(math.radians(90), 0, 0)
        )
        hoop = bpy.context.active_object
        hoop.name = f"Basketball_Hoop_{side}"

        mat_metal = create_material(f"Hoop_Mat_{side}", (1.0, 0.5, 0.0))
        if hoop.data.materials:
            hoop.data.materials[0] = mat_metal
        else:
            hoop.data.materials.append(mat_metal)

    # 舞蹈/韻律教室 (50 m²)
    create_room("舞蹈教室", sports_x, 5, 10, 5,
                floor_material=mat_sports, wall_material=mat_wall)

    # 體適能區 (20 m²)
    create_room("體適能區", sports_x - 7, 5, 5, 4,
                floor_material=mat_sports, wall_material=mat_wall)

    # ==================== 學習創作區 (150 m²) ====================
    print("\n📝 步驟 5: 建立學習創作區 (150 m²)...")
    learning_x = 6.0

    # 自習室 (60 m²) - 40個座位
    create_room("自習室", learning_x, -5, 10, 6,
                floor_material=mat_learning, wall_material=mat_wall)

    # 添加自習桌
    mat_desk = create_material("Material_Desk", (0.8, 0.7, 0.6))
    for i in range(40):
        row = i // 10
        col = i % 10
        dx = learning_x - 4 + col * 0.9
        dy = -7 + row * 0.9

        bpy.ops.mesh.primitive_cube_add(location=(dx, dy, 0.75))
        desk = bpy.context.active_object
        desk.name = f"Study_Desk_{i+1}"
        desk.scale = (0.35, 0.3, 0.025)

        if desk.data.materials:
            desk.data.materials[0] = mat_desk
        else:
            desk.data.materials.append(mat_desk)

    # 電腦教室 (50 m²) - 20台電腦
    create_room("電腦教室", learning_x, 0, 10, 5,
                floor_material=mat_learning, wall_material=mat_wall)

    # 創客空間 (40 m²)
    create_room("創客空間", learning_x, 5, 8, 5,
                floor_material=mat_learning, wall_material=mat_wall)

    # ==================== 社交娛樂區 (80 m²) ====================
    print("\n📝 步驟 6: 建立社交娛樂區 (80 m²)...")
    social_x = 10.0
    social_y = -8.0

    # 交誼廳 (50 m²)
    create_room("交誼廳", social_x, social_y, 10, 5,
                floor_material=mat_social, wall_material=mat_wall)

    # 添加沙發區（簡化為長方體）
    mat_sofa = create_material("Material_Sofa", (0.3, 0.4, 0.6))
    for i in range(3):
        bpy.ops.mesh.primitive_cube_add(location=(social_x - 3 + i * 3, social_y, 0.4))
        sofa = bpy.context.active_object
        sofa.name = f"Sofa_{i+1}"
        sofa.scale = (1.2, 0.8, 0.4)

        if sofa.data.materials:
            sofa.data.materials[0] = mat_sofa
        else:
            sofa.data.materials.append(mat_sofa)

    # 團練室 (30 m²) - 樂團排練
    create_room("團練室", social_x + 5, social_y + 6, 6, 5,
                floor_material=mat_social, wall_material=mat_wall)

    # ==================== 支援空間 (50 m²) ====================
    print("\n📝 步驟 7: 建立支援空間...")
    support_x = -12.0

    # 輔導辦公室 (20 m²)
    create_room("輔導辦公室", support_x, 0, 5, 4,
                floor_material=mat_floor, wall_material=mat_wall)

    # 器材室 (10 m²)
    create_room("器材室", support_x, -5, 4, 2.5,
                floor_material=mat_floor, wall_material=mat_wall)

    # ==================== 照明 ====================
    print("\n📝 步驟 8: 建立照明...")

    # 籃球場高亮照明
    for i in range(6):
        row = i // 3
        col = i % 3
        lx = sports_x - 5 + col * 5
        ly = -6 + row * 6

        bpy.ops.object.light_add(type='AREA', location=(lx, ly, basketball_height - 0.5))
        light = bpy.context.active_object
        light.name = f"Basketball_Light_{i+1}"
        light.data.energy = 800
        light.data.size = 3.0

    # 其他區域照明
    rooms_to_light = [
        ("舞蹈", sports_x, 5, 400),
        ("自習室", learning_x, -5, 500),
        ("電腦", learning_x, 0, 450),
        ("交誼廳", social_x, social_y, 350),
    ]

    for room_name, lx, ly, intensity in rooms_to_light:
        bpy.ops.object.light_add(type='AREA', location=(lx, ly, FLOOR_HEIGHT - 0.5))
        light = bpy.context.active_object
        light.name = f"{room_name}_Light"
        light.data.energy = intensity
        light.data.size = 3.0

    # ==================== 攝影機 ====================
    print("\n📝 步驟 9: 建立攝影機...")
    bpy.ops.object.camera_add(location=(30, -25, 18))
    camera = bpy.context.active_object
    camera.name = "Camera_Overview"
    camera.rotation_euler = (math.radians(60), 0, math.radians(50))
    bpy.context.scene.camera = camera

    # 籃球場專用攝影機
    bpy.ops.object.camera_add(location=(sports_x, -15, 4))
    basketball_cam = bpy.context.active_object
    basketball_cam.name = "Camera_Basketball"
    basketball_cam.rotation_euler = (math.radians(80), 0, 0)

    print("\n" + "="*60)
    print("  ✅ 4F 青少年活動中心建立完成！")
    print("="*60 + "\n")

    print("📊 統計:")
    print("   - 籃球場: 15m × 10m (天花板 6m)")
    print("   - 自習座位: 40個")
    print("   - 沙發區: 3組")
    print("   - 籃框: 2個")
    print("\n🎬 下一步: 匯出 USD 並儲存專案\n")

if __name__ == "__main__":
    build_floor_4F()
