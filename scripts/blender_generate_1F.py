#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - 1F Blender 建模腳本

使用方式:
1. 開啟 Blender
2. Scripting 工作區
3. 複製此腳本並執行
4. File → Export → Universal Scene Description (.usdc)

或命令列執行:
blender --background --python blender_generate_1F.py
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
def create_wall(name, start, end, height=4.0, thickness=0.2, material=None):
    """建立牆壁（從起點到終點）"""
    # 計算長度和中心點
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.sqrt(dx**2 + dy**2)

    center_x = (start[0] + end[0]) / 2
    center_y = (start[1] + end[1]) / 2
    center_z = height / 2

    # 計算旋轉角度
    angle = math.atan2(dy, dx)

    # 建立立方體
    bpy.ops.mesh.primitive_cube_add(location=(center_x, center_y, center_z))
    wall = bpy.context.active_object
    wall.name = name

    # 設定尺寸
    wall.scale = (length / 2, thickness / 2, height / 2)

    # 旋轉
    wall.rotation_euler = (0, 0, angle)

    # 套用材質
    if material:
        if wall.data.materials:
            wall.data.materials[0] = material
        else:
            wall.data.materials.append(material)

    return wall

# ==================== 建立房間 ====================
def create_room(name, x, y, width, length, height=4.0, floor_material=None, wall_material=None):
    """建立房間（地板 + 四面牆）"""

    # 建立集合（Collection）
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
        (f"{name}_Wall_South", (x - half_w, y - half_l, 0), (x + half_w, y - half_l, 0)),
        (f"{name}_Wall_North", (x - half_w, y + half_l, 0), (x + half_w, y + half_l, 0)),
        (f"{name}_Wall_West", (x - half_w, y - half_l, 0), (x - half_w, y + half_l, 0)),
        (f"{name}_Wall_East", (x + half_w, y - half_l, 0), (x + half_w, y + half_l, 0)),
    ]

    for wall_name, start, end in walls_data:
        wall = create_wall(wall_name, start, end, height, material=wall_material)
        collection.objects.link(wall)
        bpy.context.scene.collection.objects.unlink(wall)

    print(f"✅ 房間已建立: {name} ({width}m × {length}m)")

    return collection

# ==================== 主要建模函數 ====================
def build_floor_1F():
    """建立 1F 長照日照中心"""

    print("\n" + "="*60)
    print("  開始建立: 1F 長照日照中心")
    print("="*60 + "\n")

    # 1. 清空場景並設定單位
    clear_scene()
    setup_units()

    # 2. 建立材質
    print("\n📝 步驟 1: 建立材質...")
    mat_wall = create_material("Material_Wall", (0.95, 0.95, 0.9))
    mat_floor = create_material("Material_Floor", (0.85, 0.85, 0.8))
    mat_dementia = create_material("Material_Dementia", (1.0, 0.9, 0.9))
    mat_general = create_material("Material_General", (0.95, 0.98, 1.0))
    mat_dining = create_material("Material_Dining", (1.0, 0.98, 0.9))
    mat_kitchen = create_material("Material_Kitchen", (0.9, 0.9, 0.85))
    mat_support = create_material("Material_Support", (0.9, 1.0, 0.9))

    # 3. 建立主地板
    print("\n📝 步驟 2: 建立主地板...")
    FLOOR_WIDTH = 35.0
    FLOOR_LENGTH = 23.0

    main_floor = create_box(
        "Main_Floor",
        location=(0, 0, 0.025),
        scale=(FLOOR_WIDTH/2, FLOOR_LENGTH/2, 0.025),
        material=mat_floor
    )

    # 4. 建立外牆
    print("\n📝 步驟 3: 建立外牆...")
    half_w = FLOOR_WIDTH / 2
    half_l = FLOOR_LENGTH / 2
    FLOOR_HEIGHT = 4.0

    exterior_walls = [
        ("Exterior_South", (-half_w, -half_l, 0), (half_w, -half_l, 0)),
        ("Exterior_North", (-half_w, half_l, 0), (half_w, half_l, 0)),
        ("Exterior_West", (-half_w, -half_l, 0), (-half_w, half_l, 0)),
        ("Exterior_East", (half_w, -half_l, 0), (half_w, half_l, 0)),
    ]

    for name, start, end in exterior_walls:
        create_wall(name, start, end, FLOOR_HEIGHT, thickness=0.30, material=mat_wall)

    # 5. 建立失智專區 (200 m²)
    print("\n📝 步驟 4: 建立失智專區...")
    dementia_x = -10.0

    create_room("失智_安靜活動室", dementia_x, -5, 10, 8,
                floor_material=mat_dementia, wall_material=mat_wall)

    create_room("失智_感官刺激室", dementia_x, 5, 8, 7.5,
                floor_material=mat_dementia, wall_material=mat_wall)

    # 6. 建立一般日照區 (300 m²)
    print("\n📝 步驟 5: 建立一般日照區...")
    general_x = 5.0

    create_room("一般_團體活動室", general_x, -3, 15, 10,
                floor_material=mat_general, wall_material=mat_wall)

    create_room("一般_復健訓練室", general_x + 8, 6, 10, 8,
                floor_material=mat_general, wall_material=mat_wall)

    create_room("一般_休息室", general_x - 8, 6, 10, 7,
                floor_material=mat_general, wall_material=mat_wall)

    # 7. 建立共享空間 (220 m²)
    print("\n📝 步驟 6: 建立共享空間...")
    shared_x = 12.0

    create_room("共用餐廳", shared_x, 0, 12, 10,
                floor_material=mat_dining, wall_material=mat_wall)

    create_room("備餐廚房", shared_x + 8, 7, 8, 5,
                floor_material=mat_kitchen, wall_material=mat_wall)

    # 8. 建立支援空間
    print("\n📝 步驟 7: 建立支援空間...")
    support_x = -15.0

    create_room("護理站", support_x, 0, 5, 5,
                floor_material=mat_support, wall_material=mat_wall)

    # 9. 添加照明
    print("\n📝 步驟 8: 建立照明...")

    # 太陽光
    bpy.ops.object.light_add(type='SUN', location=(0, 0, 20))
    sun = bpy.context.active_object
    sun.name = "Sun"
    sun.data.energy = 3.0
    sun.rotation_euler = (math.radians(45), 0, math.radians(135))

    # 區域光（每個主要房間）
    lights_data = [
        ("失智區光源", dementia_x, -5),
        ("一般區光源", general_x, -3),
        ("餐廳光源", shared_x, 0),
    ]

    for light_name, lx, ly in lights_data:
        bpy.ops.object.light_add(type='AREA', location=(lx, ly, FLOOR_HEIGHT - 0.5))
        light = bpy.context.active_object
        light.name = light_name
        light.data.energy = 500
        light.data.size = 3.0

    # 10. 設定攝影機
    print("\n📝 步驟 9: 建立攝影機...")
    bpy.ops.object.camera_add(location=(40, -30, 15))
    camera = bpy.context.active_object
    camera.name = "Camera_Overview"
    camera.rotation_euler = (math.radians(60), 0, math.radians(45))

    # 設為場景攝影機
    bpy.context.scene.camera = camera

    print("\n" + "="*60)
    print("  ✅ 1F 長照日照中心建立完成！")
    print("="*60 + "\n")

    print("🎬 下一步:")
    print("   1. 調整視角: 使用數字鍵盤 0 切換到攝影機視角")
    print("   2. 渲染預覽: Viewport Shading → Rendered (Z 鍵選單)")
    print("   3. 匯出 USD: File → Export → USD (.usdc)")
    print("   4. 儲存 Blender 檔: File → Save As → floor_1F.blend\n")

# ==================== 執行 ====================
if __name__ == "__main__":
    build_floor_1F()
