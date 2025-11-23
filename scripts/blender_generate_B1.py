#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - B1 地下層建模腳本
停車場 + 設備層

使用方式:
1. 開啟 Blender
2. Scripting 工作區
3. 複製此腳本並執行
4. File → Export → Universal Scene Description (.usdc)
"""

import bpy
import math

# ==================== 清空場景 ====================
def clear_scene():
    """刪除所有現有物件"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    print("✅ 場景已清空")

# ==================== 設定單位 ====================
def setup_units():
    """設定公制單位"""
    bpy.context.scene.unit_settings.system = 'METRIC'
    bpy.context.scene.unit_settings.length_unit = 'METERS'
    print("✅ 單位設定: 公尺")

# ==================== 建立材質 ====================
def create_material(name, color):
    """建立簡單材質"""
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs['Base Color'].default_value = (*color, 1.0)
        bsdf.inputs['Roughness'].default_value = 0.5
    return mat

# ==================== 建立立方體 ====================
def create_box(name, location, scale, material=None):
    """建立立方體並套用材質"""
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

# ==================== 建立牆壁 ====================
def create_wall(name, start, end, height=3.5, thickness=0.2, material=None):
    """建立牆壁（從起點到終點）"""
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

# ==================== 建立停車位 ====================
def create_parking_space(name, x, y, width=2.5, length=5.0, is_accessible=False, material=None):
    """建立單個停車位（地面標線）"""

    # 停車位地板
    bpy.ops.mesh.primitive_plane_add(location=(x, y, 0.01))
    space = bpy.context.active_object
    space.name = f"{name}_Floor"
    space.scale = (width/2, length/2, 1)

    if material:
        if space.data.materials:
            space.data.materials[0] = material
        else:
            space.data.materials.append(material)

    # 如果是無障礙車位，添加標示
    if is_accessible:
        # 添加輪椅符號（簡化為圓形+矩形）
        bpy.ops.mesh.primitive_cylinder_add(location=(x, y, 0.02), radius=0.3, depth=0.01)
        symbol = bpy.context.active_object
        symbol.name = f"{name}_Symbol"

        # 白色材質
        mat_white = create_material(f"{name}_White", (1.0, 1.0, 1.0))
        if symbol.data.materials:
            symbol.data.materials[0] = mat_white
        else:
            symbol.data.materials.append(mat_white)

    return space

# ==================== 建立房間 ====================
def create_room(name, x, y, width, length, height=3.5, floor_material=None, wall_material=None):
    """建立房間（地板 + 四面牆）"""

    collection = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(collection)

    # 地板
    floor = create_box(
        f"{name}_Floor",
        location=(x, y, 0.025),
        scale=(width/2, length/2, 0.025),
        material=floor_material
    )
    collection.objects.link(floor)
    bpy.context.scene.collection.objects.unlink(floor)

    # 四面牆
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
def build_floor_B1():
    """建立 B1 地下層：停車場 + 設備層"""

    print("\n" + "="*60)
    print("  開始建立: B1 地下層（停車場+設備層）")
    print("="*60 + "\n")

    # 1. 清空場景並設定單位
    clear_scene()
    setup_units()

    # 2. 建立材質
    print("\n📝 步驟 1: 建立材質...")
    mat_concrete = create_material("Material_Concrete", (0.6, 0.6, 0.6))
    mat_wall = create_material("Material_Wall", (0.8, 0.8, 0.75))
    mat_parking_general = create_material("Material_Parking_General", (0.5, 0.5, 0.55))
    mat_parking_accessible = create_material("Material_Parking_Accessible", (0.7, 0.9, 1.0))
    mat_parking_family = create_material("Material_Parking_Family", (0.9, 1.0, 0.7))
    mat_equipment = create_material("Material_Equipment", (0.85, 0.85, 0.8))

    # 3. 建立主地板
    print("\n📝 步驟 2: 建立主地板...")
    FLOOR_WIDTH = 30.0
    FLOOR_LENGTH = 20.0
    FLOOR_HEIGHT = 3.5

    main_floor = create_box(
        "Main_Floor",
        location=(0, 0, 0.025),
        scale=(FLOOR_WIDTH/2, FLOOR_LENGTH/2, 0.025),
        material=mat_concrete
    )

    # 4. 建立外牆
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
        create_wall(name, start, end, FLOOR_HEIGHT, thickness=0.40, material=mat_wall)

    # 5. 建立停車區域 (450 m²)
    print("\n📝 步驟 4: 建立停車區域...")

    # 車道入口（左側）
    entrance_x = -12.0
    entrance_y = -8.0

    # 一般停車位 (20格, 2.5m × 5.0m)
    print("   - 建立一般停車位 (20格)...")
    parking_start_x = -10.0
    parking_start_y = -6.0

    for i in range(20):
        row = i // 5
        col = i % 5
        x = parking_start_x + col * 3.0  # 2.5m車位 + 0.5m間距
        y = parking_start_y + row * 6.0  # 5.0m車位 + 1.0m通道
        create_parking_space(f"Parking_General_{i+1}", x, y,
                            width=2.5, length=5.0, material=mat_parking_general)

    # 無障礙停車位 (5格, 3.5m × 5.0m)
    print("   - 建立無障礙停車位 (5格)...")
    accessible_x = 5.0
    accessible_y = -6.0

    for i in range(5):
        x = accessible_x + i * 4.0  # 3.5m車位 + 0.5m間距
        y = accessible_y
        create_parking_space(f"Parking_Accessible_{i+1}", x, y,
                            width=3.5, length=5.0, is_accessible=True,
                            material=mat_parking_accessible)

    # 親子優先停車位 (5格, 2.5m × 5.5m)
    print("   - 建立親子優先停車位 (5格)...")
    family_x = 5.0
    family_y = 2.0

    for i in range(5):
        x = family_x + i * 3.2  # 2.5m車位 + 0.7m間距
        y = family_y
        create_parking_space(f"Parking_Family_{i+1}", x, y,
                            width=2.5, length=5.5, material=mat_parking_family)

    # 6. 建立設備層 (150 m²)
    print("\n📝 步驟 5: 建立設備層...")
    equipment_x = -8.0
    equipment_y = 6.0

    # 空調機房 (30 m²)
    create_room("設備_空調機房", equipment_x - 5, equipment_y, 6, 5,
                floor_material=mat_equipment, wall_material=mat_wall)

    # 配電室 (20 m²)
    create_room("設備_配電室", equipment_x + 2, equipment_y, 5, 4,
                floor_material=mat_equipment, wall_material=mat_wall)

    # 給排水設備 (25 m²)
    create_room("設備_給排水", equipment_x + 8, equipment_y, 5, 5,
                floor_material=mat_equipment, wall_material=mat_wall)

    # 倉儲空間 (60 m²)
    create_room("設備_倉儲空間", equipment_x + 5, equipment_y + 7, 10, 6,
                floor_material=mat_equipment, wall_material=mat_wall)

    # 7. 建立電梯井與樓梯
    print("\n📝 步驟 6: 建立垂直動線...")

    # 客梯1 (2m × 2m)
    create_room("電梯1", 0, 8, 2, 2, height=FLOOR_HEIGHT,
                floor_material=mat_concrete, wall_material=mat_wall)

    # 客梯2 (2m × 2m)
    create_room("電梯2", 3, 8, 2, 2, height=FLOOR_HEIGHT,
                floor_material=mat_concrete, wall_material=mat_wall)

    # 安全梯 (1.2m寬，環形設計)
    create_room("安全梯", -3, 8, 3, 3, height=FLOOR_HEIGHT,
                floor_material=mat_concrete, wall_material=mat_wall)

    # 8. 添加簡單設備模型
    print("\n📝 步驟 7: 添加設備模型...")

    # 空調主機 (簡化為長方體)
    bpy.ops.mesh.primitive_cube_add(location=(equipment_x - 5, equipment_y, 1.5))
    hvac = bpy.context.active_object
    hvac.name = "HVAC_Unit"
    hvac.scale = (2, 1.5, 1.5)

    mat_metal = create_material("Material_Metal", (0.7, 0.7, 0.75))
    if hvac.data.materials:
        hvac.data.materials[0] = mat_metal
    else:
        hvac.data.materials.append(mat_metal)

    # 9. 添加照明
    print("\n📝 步驟 8: 建立照明...")

    # 停車場區域光（冷白光）
    for i in range(6):
        row = i // 3
        col = i % 3
        lx = -8 + col * 8
        ly = -6 + row * 8

        bpy.ops.object.light_add(type='AREA', location=(lx, ly, FLOOR_HEIGHT - 0.3))
        light = bpy.context.active_object
        light.name = f"Parking_Light_{i+1}"
        light.data.energy = 300
        light.data.size = 2.0
        light.data.color = (0.9, 0.95, 1.0)  # 冷白光

    # 設備層照明
    bpy.ops.object.light_add(type='AREA', location=(equipment_x, equipment_y, FLOOR_HEIGHT - 0.3))
    light = bpy.context.active_object
    light.name = "Equipment_Light"
    light.data.energy = 400
    light.data.size = 3.0

    # 10. 設定攝影機
    print("\n📝 步驟 9: 建立攝影機...")
    bpy.ops.object.camera_add(location=(25, -20, 10))
    camera = bpy.context.active_object
    camera.name = "Camera_Overview"
    camera.rotation_euler = (math.radians(65), 0, math.radians(50))
    bpy.context.scene.camera = camera

    print("\n" + "="*60)
    print("  ✅ B1 地下層建立完成！")
    print("="*60 + "\n")

    print("🎬 下一步:")
    print("   1. 調整視角: 數字鍵 0 切換攝影機視角")
    print("   2. 渲染預覽: Z → Rendered")
    print("   3. 匯出 USD: File → Export → USD (.usdc)")
    print("   4. 儲存檔案: File → Save As → floor_B1.blend\n")

# ==================== 執行 ====================
if __name__ == "__main__":
    build_floor_B1()
