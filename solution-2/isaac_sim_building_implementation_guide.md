# 赤土崎全齡社福樞紐 - NVIDIA Isaac Sim 3D建築實作指南

## 📋 專案總覽

### 建築物資訊摘要
- **專案名稱**: 赤土崎多功能館建築設計方案
- **建築規模**: 地下1層 + 地上4層（B1+4F）
- **總樓地板面積**: 3,100 m²
- **總服務人數**: 140-180人
- **建築高度**: 約18-20公尺

### 各樓層功能配置
| 樓層 | 主要功能 | 面積 | 重點設施 |
|------|---------|------|----------|
| B1 | 停車場+設備層 | 600 m² | 30車位、機房設備 |
| 1F | 長照日照中心 | 800 m² | 失智專區、復健室、庭園 |
| 2F | 公共托嬰中心 | 700 m² | 嬰幼兒室、戶外遊戲區 |
| 3F | 家庭支持服務 | 500 m² | 多功能教室、社區廚房 |
| 4F | 青少年活動中心 | 500 m² | 籃球場、創客空間、自習室 |

---

## 🎯 實作策略

### 第一階段：環境設置與準備（第1-2週）

#### 1.1 系統需求
```bash
# 最低硬體需求
- GPU: NVIDIA RTX 5090 (你的配置) ✓
- RAM: 32GB 以上
- 儲存空間: 500GB SSD
- 作業系統: Ubuntu 20.04 或 Windows 10/11

# 軟體需求
- NVIDIA Isaac Sim 2023.1.1 或更新版本
- Python 3.10+
- Blender 3.6+ (用於模型製作)
- USD Composer (內建於 Isaac Sim)
```

#### 1.2 安裝 Isaac Sim
```bash
# 透過 Omniverse Launcher 安裝
1. 下載 NVIDIA Omniverse Launcher
2. 安裝 Isaac Sim
3. 安裝額外套件：
   - Replicator
   - PhysX
   - RTX Real-Time Renderer
```

#### 1.3 專案結構設置
```
easterlin-hsinchu-3d/
├── assets/                 # 3D 模型資產
│   ├── architecture/       # 建築結構
│   ├── furniture/         # 家具設施
│   ├── equipment/         # 設備器材
│   └── characters/        # 人物模型
├── scenes/                # 場景檔案
│   ├── b1_parking.usd
│   ├── 1f_elderly_care.usd
│   ├── 2f_nursery.usd
│   ├── 3f_family_support.usd
│   └── 4f_youth_center.usd
├── scripts/               # Python 腳本
│   ├── building_generator.py
│   ├── facility_placer.py
│   └── simulation_manager.py
├── configs/               # 配置檔案
│   └── floor_configs.yaml
└── simulations/          # 模擬場景
```

---

### 第二階段：建築結構建模（第3-4週）

#### 2.1 基礎建築框架生成腳本
```python
# scripts/building_generator.py
import omni
from pxr import Gf, UsdGeom, Sdf
import numpy as np

class BuildingGenerator:
    def __init__(self, stage_path="/World/Building"):
        self.stage = omni.usd.get_context().get_stage()
        self.stage_path = stage_path
        
    def create_floor_structure(self, floor_name, floor_height, floor_area):
        """建立單一樓層結構"""
        floor_path = f"{self.stage_path}/{floor_name}"
        floor_prim = UsdGeom.Xform.Define(self.stage, floor_path)
        
        # 建立樓板
        floor_mesh = UsdGeom.Cube.Define(
            self.stage, 
            f"{floor_path}/floor_plate"
        )
        floor_mesh.GetExtentAttr().Set([
            (-floor_area[0]/2, -floor_area[1]/2, -0.15),
            (floor_area[0]/2, floor_area[1]/2, 0.15)
        ])
        
        # 建立牆壁
        self.create_walls(floor_path, floor_area, floor_height)
        
        # 建立柱子
        self.create_columns(floor_path, floor_area)
        
        return floor_prim
    
    def create_walls(self, floor_path, area, height):
        """建立牆壁系統"""
        wall_thickness = 0.2  # 20cm 厚度
        
        # 外牆配置
        walls = [
            ("north", [0, area[1]/2, height/2], [area[0], wall_thickness, height]),
            ("south", [0, -area[1]/2, height/2], [area[0], wall_thickness, height]),
            ("east", [area[0]/2, 0, height/2], [wall_thickness, area[1], height]),
            ("west", [-area[0]/2, 0, height/2], [wall_thickness, area[1], height])
        ]
        
        for wall_name, position, size in walls:
            wall_path = f"{floor_path}/walls/{wall_name}"
            wall = UsdGeom.Cube.Define(self.stage, wall_path)
            wall.GetExtentAttr().Set([
                (-size[0]/2, -size[1]/2, -size[2]/2),
                (size[0]/2, size[1]/2, size[2]/2)
            ])
            UsdGeom.XformCommonAPI(wall).SetTranslate(position)
    
    def create_columns(self, floor_path, area):
        """建立結構柱"""
        column_size = 0.6  # 60cm x 60cm
        column_spacing = 8.0  # 8公尺間距
        
        cols_x = int(area[0] / column_spacing) + 1
        cols_y = int(area[1] / column_spacing) + 1
        
        for i in range(cols_x):
            for j in range(cols_y):
                x = -area[0]/2 + i * column_spacing
                y = -area[1]/2 + j * column_spacing
                
                column_path = f"{floor_path}/columns/col_{i}_{j}"
                column = UsdGeom.Cube.Define(self.stage, column_path)
                column.GetSizeAttr().Set(column_size)
                UsdGeom.XformCommonAPI(column).SetTranslate([x, y, 0])

# 使用範例
def generate_building():
    generator = BuildingGenerator()
    
    # 建築物規格（根據文件）
    floor_specs = {
        "B1": {"height": 3.0, "area": [30, 20], "z_offset": -3.0},
        "1F": {"height": 3.5, "area": [40, 20], "z_offset": 0},
        "2F": {"height": 3.5, "area": [35, 20], "z_offset": 3.5},
        "3F": {"height": 3.5, "area": [25, 20], "z_offset": 7.0},
        "4F": {"height": 4.0, "area": [25, 20], "z_offset": 10.5}
    }
    
    for floor_name, specs in floor_specs.items():
        floor_prim = generator.create_floor_structure(
            floor_name, 
            specs["height"], 
            specs["area"]
        )
        # 設定樓層高度
        UsdGeom.XformCommonAPI(floor_prim).SetTranslate(
            [0, 0, specs["z_offset"]]
        )
```

#### 2.2 各樓層空間劃分配置
```yaml
# configs/floor_configs.yaml

b1_parking:
  spaces:
    - name: "parking_area"
      type: "parking"
      area: 450
      position: [0, 0, -3]
      features:
        - parking_spots: 30
        - handicapped_spots: 5
        - family_spots: 5
    - name: "equipment_room"
      type: "utility"
      area: 150
      position: [15, 0, -3]
      equipment:
        - hvac_system
        - electrical_room
        - water_system
        - emergency_generator

floor_1_elderly:
  spaces:
    - name: "dementia_area"
      type: "care_unit"
      area: 200
      position: [-10, 0, 0]
      features:
        - quiet_activity_room: 80
        - sensory_room: 60
        - wandering_corridor: 60
      soundproofing: "STC_65"
    - name: "general_daycare"
      type: "activity_area"
      area: 300
      position: [0, 0, 0]
      features:
        - group_activity: 150
        - rehabilitation: 80
        - rest_area: 70
    - name: "shared_dining"
      type: "dining"
      area: 120
      position: [10, 0, 0]
      capacity: 60
    - name: "accessible_garden"
      type: "outdoor"
      area: 60
      position: [15, 10, 0]
      features:
        - raised_planters
        - sensory_plants
        - shaded_seating

floor_2_nursery:
  spaces:
    - name: "infant_room"
      type: "nursery"
      age_group: "0-1"
      area: 180
      position: [-8, 0, 3.5]
      features:
        - play_area: 80
        - sleeping_room: 60
        - milk_prep: 20
        - diaper_station: 20
      soundproofing: "IIC_65"
    - name: "toddler_room"
      type: "nursery"
      age_group: "1-2"
      area: 250
      position: [5, 0, 3.5]
      features:
        - play_area: 120
        - nap_room: 80
        - reading_corner: 30
        - sensory_integration: 20
    - name: "outdoor_play"
      type: "outdoor"
      area: 85
      position: [12, 8, 3.5]
      features:
        - sandbox
        - climbing_structures
        - water_play

floor_3_family:
  spaces:
    - name: "multipurpose_room"
      type: "community"
      area: 150
      position: [0, 0, 7]
      features:
        - movable_partitions
        - projection_system
        - sound_system
    - name: "community_kitchen"
      type: "kitchen"
      area: 80
      position: [8, 0, 7]
      equipment:
        - commercial_stove
        - prep_stations
        - dishwasher
    - name: "consultation_rooms"
      type: "office"
      area: 60
      position: [-8, 0, 7]
      rooms: 3
      features:
        - soundproofing
        - comfortable_seating

floor_4_youth:
  spaces:
    - name: "basketball_court"
      type: "sports"
      area: 150
      position: [0, 0, 10.5]
      features:
        - half_court
        - adjustable_hoops
        - pu_flooring
      soundproofing: "IIC_70"
    - name: "dance_studio"
      type: "activity"
      area: 50
      position: [-10, 0, 10.5]
      features:
        - mirror_walls
        - ballet_barres
        - sound_system
    - name: "study_room"
      type: "educational"
      area: 60
      position: [8, 5, 10.5]
      capacity: 40
      features:
        - individual_desks
        - group_tables
        - quiet_zones
    - name: "maker_space"
      type: "creative"
      area: 40
      position: [8, -5, 10.5]
      equipment:
        - 3d_printers: 2
        - laser_cutter: 1
        - electronics_kits
```

---

### 第三階段：設施與家具配置（第5-6週）

#### 3.1 設施擺放系統
```python
# scripts/facility_placer.py
import omni
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
import yaml
import random

class FacilityPlacer:
    def __init__(self, config_path):
        self.stage = omni.usd.get_context().get_stage()
        self.assets_path = get_assets_root_path() + "/Isaac/Environments/Hospital"
        
        # 載入配置
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # 預設資產庫
        self.asset_library = {
            # 長照設施
            "wheelchair": "/Props/Wheelchair.usd",
            "hospital_bed": "/Props/HospitalBed.usd",
            "walker": "/Props/Walker.usd",
            
            # 托嬰設施
            "crib": "/Props/Crib.usd",
            "changing_table": "/Props/ChangingTable.usd",
            "play_mat": "/Props/PlayMat.usd",
            
            # 青少年設施
            "basketball_hoop": "/Props/BasketballHoop.usd",
            "desk": "/Props/Desk.usd",
            "computer": "/Props/Computer.usd",
            
            # 通用設施
            "chair": "/Props/Chair.usd",
            "table": "/Props/Table.usd",
            "sofa": "/Props/Sofa.usd",
            "cabinet": "/Props/Cabinet.usd"
        }
    
    def place_floor_facilities(self, floor_name):
        """為特定樓層配置設施"""
        floor_config = self.config.get(floor_name, {})
        
        for space in floor_config.get("spaces", []):
            self.place_space_facilities(floor_name, space)
    
    def place_space_facilities(self, floor_name, space_config):
        """為特定空間配置設施"""
        space_name = space_config["name"]
        space_type = space_config["type"]
        position = space_config["position"]
        
        # 根據空間類型選擇設施
        facilities = self.get_facilities_by_type(space_type)
        
        # 配置設施
        for facility_type, count in facilities.items():
            for i in range(count):
                self.place_single_facility(
                    floor_name,
                    space_name, 
                    facility_type,
                    position,
                    i
                )
    
    def get_facilities_by_type(self, space_type):
        """根據空間類型返回所需設施"""
        facility_mapping = {
            "care_unit": {
                "wheelchair": 5,
                "walker": 3,
                "chair": 20,
                "table": 10
            },
            "nursery": {
                "crib": 15,
                "changing_table": 2,
                "play_mat": 5
            },
            "sports": {
                "basketball_hoop": 2
            },
            "educational": {
                "desk": 30,
                "chair": 30,
                "computer": 20
            },
            "dining": {
                "table": 10,
                "chair": 60
            },
            "parking": {
                # 停車場可以加入車輛模型
            }
        }
        
        return facility_mapping.get(space_type, {})
    
    def place_single_facility(self, floor, space, facility_type, 
                            base_position, index):
        """放置單一設施"""
        asset_path = self.asset_library.get(facility_type)
        if not asset_path:
            return
        
        # 計算位置（加入一些隨機偏移避免重疊）
        offset_x = (index % 5) * 2.0 + random.uniform(-0.5, 0.5)
        offset_y = (index // 5) * 2.0 + random.uniform(-0.5, 0.5)
        
        position = [
            base_position[0] + offset_x,
            base_position[1] + offset_y,
            base_position[2]
        ]
        
        # 建立設施物件
        prim_path = f"/World/Building/{floor}/{space}/{facility_type}_{index}"
        
        # 加入 USD 參考
        prim = self.stage.DefinePrim(prim_path)
        prim.GetReferences().AddReference(self.assets_path + asset_path)
        
        # 設定位置
        xform = XFormPrim(prim_path)
        xform.set_world_pose(position=position)
        
        print(f"放置 {facility_type} 於 {floor}/{space}")

# 執行設施配置
def setup_all_facilities():
    placer = FacilityPlacer("configs/floor_configs.yaml")
    
    floors = ["b1_parking", "floor_1_elderly", "floor_2_nursery", 
              "floor_3_family", "floor_4_youth"]
    
    for floor in floors:
        placer.place_floor_facilities(floor)
```

---

### 第四階段：物理模擬與互動（第7-8週）

#### 4.1 物理屬性設定
```python
# scripts/physics_setup.py
from omni.isaac.core.physics_context import PhysicsContext
from omni.isaac.core.prims import RigidPrimView
from pxr import UsdPhysics, PhysxSchema

class PhysicsSetup:
    def __init__(self):
        self.physics_context = PhysicsContext()
        self.setup_physics_scene()
    
    def setup_physics_scene(self):
        """設定物理場景參數"""
        self.physics_context.set_gravity(9.81)
        self.physics_context.set_solver_type("TGS")
        
        # 設定物理材質
        self.setup_materials()
        
        # 設定碰撞層
        self.setup_collision_layers()
    
    def setup_materials(self):
        """設定不同材質的物理屬性"""
        materials = {
            "concrete": {
                "static_friction": 0.9,
                "dynamic_friction": 0.7,
                "restitution": 0.2
            },
            "wood": {
                "static_friction": 0.5,
                "dynamic_friction": 0.3,
                "restitution": 0.4
            },
            "rubber": {
                "static_friction": 1.0,
                "dynamic_friction": 0.8,
                "restitution": 0.8
            },
            "metal": {
                "static_friction": 0.4,
                "dynamic_friction": 0.3,
                "restitution": 0.1
            }
        }
        
        for material_name, properties in materials.items():
            self.create_physics_material(material_name, properties)
    
    def create_physics_material(self, name, properties):
        """建立物理材質"""
        material_path = f"/World/Materials/{name}"
        material = UsdPhysics.MaterialAPI.Apply(
            self.stage.DefinePrim(material_path)
        )
        
        # 設定摩擦力和彈性
        material.CreateStaticFrictionAttr(properties["static_friction"])
        material.CreateDynamicFrictionAttr(properties["dynamic_friction"])
        material.CreateRestitutionAttr(properties["restitution"])
    
    def setup_collision_layers(self):
        """設定碰撞層（用於不同物件間的互動）"""
        layers = {
            "static_building": 0,      # 建築結構
            "furniture": 1,             # 家具
            "characters": 2,            # 人物
            "wheelchairs": 3,           # 輪椅
            "toys": 4,                  # 玩具
            "sports_equipment": 5       # 運動器材
        }
        
        # 設定碰撞規則
        collision_rules = [
            ("characters", "static_building", True),
            ("characters", "furniture", True),
            ("wheelchairs", "static_building", True),
            ("wheelchairs", "furniture", False),  # 輪椅可穿過某些家具
            ("toys", "static_building", True),
            ("sports_equipment", "static_building", True)
        ]
        
        for obj1, obj2, should_collide in collision_rules:
            self.set_collision_rule(
                layers[obj1], 
                layers[obj2], 
                should_collide
            )
    
    def apply_rigid_body(self, prim_path, mass=1.0, material="wood"):
        """為物件加入剛體物理"""
        prim = self.stage.GetPrimAtPath(prim_path)
        
        # 加入剛體
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body.CreateMassAttr(mass)
        
        # 加入碰撞
        collision = UsdPhysics.CollisionAPI.Apply(prim)
        
        # 指定材質
        material_path = f"/World/Materials/{material}"
        collision.GetRel("material:binding").SetTargets([material_path])
```

#### 4.2 人物動畫與行為模擬
```python
# scripts/character_simulation.py
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.rotations import quat_to_euler_angles
import numpy as np

class CharacterSimulation:
    def __init__(self):
        self.characters = {}
        self.behavior_patterns = {}
        self.setup_characters()
    
    def setup_characters(self):
        """設定不同類型的人物"""
        character_types = {
            "elderly": {
                "count": 50,
                "floor": "1F",
                "speed": 0.5,  # m/s
                "activities": ["sitting", "walking", "exercising", "eating"]
            },
            "toddler": {
                "count": 40,
                "floor": "2F", 
                "speed": 0.3,
                "activities": ["crawling", "playing", "sleeping", "eating"]
            },
            "youth": {
                "count": 30,
                "floor": "4F",
                "speed": 1.5,
                "activities": ["studying", "playing_sports", "socializing"]
            },
            "caregiver": {
                "count": 15,
                "floor": "all",
                "speed": 1.0,
                "activities": ["caring", "feeding", "monitoring", "emergency"]
            }
        }
        
        for char_type, config in character_types.items():
            self.spawn_characters(char_type, config)
    
    def spawn_characters(self, char_type, config):
        """生成特定類型的人物"""
        for i in range(config["count"]):
            char_path = f"/World/Characters/{char_type}_{i}"
            
            # 載入人物模型
            if char_type == "elderly":
                self.load_elderly_character(char_path, i)
            elif char_type == "toddler":
                self.load_toddler_character(char_path, i)
            elif char_type == "youth":
                self.load_youth_character(char_path, i)
            elif char_type == "caregiver":
                self.load_caregiver_character(char_path, i)
            
            # 設定初始位置
            floor = config["floor"]
            position = self.get_spawn_position(floor, i)
            
            self.characters[char_path] = {
                "type": char_type,
                "position": position,
                "speed": config["speed"],
                "current_activity": None,
                "schedule": self.generate_schedule(char_type)
            }
    
    def load_elderly_character(self, path, index):
        """載入長者角色（包含輔助設備）"""
        # 載入基本人物模型
        character = self.stage.DefinePrim(path)
        character.GetReferences().AddReference(
            "omniverse://localhost/NVIDIA/Assets/Characters/Elderly.usd"
        )
        
        # 20% 的長者使用輪椅
        if index % 5 == 0:
            wheelchair_path = f"{path}/wheelchair"
            wheelchair = self.stage.DefinePrim(wheelchair_path)
            wheelchair.GetReferences().AddReference(
                "omniverse://localhost/NVIDIA/Assets/Props/Wheelchair.usd"
            )
        
        # 30% 的長者使用助行器
        elif index % 3 == 0:
            walker_path = f"{path}/walker"
            walker = self.stage.DefinePrim(walker_path)
            walker.GetReferences().AddReference(
                "omniverse://localhost/NVIDIA/Assets/Props/Walker.usd"
            )
    
    def generate_schedule(self, char_type):
        """生成日程表"""
        schedules = {
            "elderly": [
                {"time": "09:00", "activity": "arrival", "location": "entrance"},
                {"time": "09:30", "activity": "exercise", "location": "activity_room"},
                {"time": "10:30", "activity": "rest", "location": "rest_area"},
                {"time": "11:30", "activity": "lunch", "location": "dining"},
                {"time": "13:00", "activity": "nap", "location": "rest_area"},
                {"time": "14:00", "activity": "rehabilitation", "location": "rehab_room"},
                {"time": "15:30", "activity": "garden", "location": "outdoor"},
                {"time": "16:30", "activity": "departure", "location": "entrance"}
            ],
            "toddler": [
                {"time": "07:30", "activity": "arrival", "location": "entrance"},
                {"time": "08:00", "activity": "play", "location": "play_area"},
                {"time": "09:30", "activity": "snack", "location": "dining"},
                {"time": "10:00", "activity": "learning", "location": "activity_room"},
                {"time": "11:30", "activity": "lunch", "location": "dining"},
                {"time": "12:30", "activity": "nap", "location": "nap_room"},
                {"time": "14:30", "activity": "play", "location": "outdoor"},
                {"time": "15:30", "activity": "snack", "location": "dining"},
                {"time": "16:30", "activity": "departure", "location": "entrance"}
            ],
            "youth": [
                {"time": "17:00", "activity": "arrival", "location": "entrance"},
                {"time": "17:30", "activity": "sports", "location": "basketball_court"},
                {"time": "18:30", "activity": "dinner", "location": "dining"},
                {"time": "19:00", "activity": "study", "location": "study_room"},
                {"time": "20:30", "activity": "social", "location": "social_area"},
                {"time": "21:00", "activity": "departure", "location": "entrance"}
            ]
        }
        
        return schedules.get(char_type, [])
    
    def simulate_behavior(self, character_path, current_time):
        """模擬角色行為"""
        char_data = self.characters[character_path]
        schedule = char_data["schedule"]
        
        # 找出當前時段的活動
        current_activity = None
        for activity in schedule:
            if self.is_time_in_range(current_time, activity["time"]):
                current_activity = activity
                break
        
        if current_activity:
            self.execute_activity(character_path, current_activity)
    
    def execute_activity(self, character_path, activity):
        """執行特定活動"""
        activity_type = activity["activity"]
        location = activity["location"]
        
        # 移動到目標位置
        target_position = self.get_location_position(location)
        self.move_character(character_path, target_position)
        
        # 執行動作
        if activity_type == "exercise":
            self.perform_exercise_animation(character_path)
        elif activity_type == "eating":
            self.perform_eating_animation(character_path)
        elif activity_type == "play":
            self.perform_play_animation(character_path)
        # ... 更多活動類型
```

---

### 第五階段：跨齡互動模擬（第9-10週）

#### 5.1 跨齡互動場景設定
```python
# scripts/intergenerational_interaction.py
import omni
from datetime import datetime, timedelta
import random

class IntergenerationalInteraction:
    def __init__(self):
        self.interaction_zones = {}
        self.scheduled_activities = []
        self.setup_interaction_zones()
        self.plan_activities()
    
    def setup_interaction_zones(self):
        """設定跨齡互動區域"""
        self.interaction_zones = {
            "garden": {
                "location": "1F_outdoor_garden",
                "participants": ["elderly", "toddler"],
                "activities": ["gardening", "nature_observation"],
                "capacity": 20
            },
            "dining_hall": {
                "location": "1F_dining",
                "participants": ["elderly", "toddler", "youth"],
                "activities": ["shared_meals", "celebration"],
                "capacity": 60
            },
            "multipurpose_room": {
                "location": "3F_multipurpose",
                "participants": ["all"],
                "activities": ["performance", "workshop", "festival"],
                "capacity": 80
            },
            "reading_corner": {
                "location": "2F_reading",
                "participants": ["elderly", "toddler"],
                "activities": ["storytelling", "reading"],
                "capacity": 15
            }
        }
    
    def plan_activities(self):
        """規劃跨齡活動時程"""
        weekly_activities = [
            {
                "name": "晨間園藝",
                "day": "Monday",
                "time": "10:00",
                "duration": 60,
                "zone": "garden",
                "participants": {
                    "elderly": 10,
                    "toddler": 10,
                    "caregiver": 3
                },
                "description": "長者教導幼兒種植蔬菜"
            },
            {
                "name": "說故事時間",
                "day": "Wednesday", 
                "time": "14:00",
                "duration": 45,
                "zone": "reading_corner",
                "participants": {
                    "elderly": 5,
                    "toddler": 10,
                    "caregiver": 2
                },
                "description": "長者為幼兒講述故事"
            },
            {
                "name": "共融午餐",
                "day": "Friday",
                "time": "11:30",
                "duration": 90,
                "zone": "dining_hall",
                "participants": {
                    "elderly": 20,
                    "toddler": 15,
                    "youth": 10,
                    "caregiver": 5
                },
                "description": "三代同堂共進午餐"
            },
            {
                "name": "才藝表演",
                "day": "Saturday",
                "time": "15:00",
                "duration": 120,
                "zone": "multipurpose_room",
                "participants": {
                    "elderly": 30,
                    "toddler": 20,
                    "youth": 20,
                    "family": 30,
                    "caregiver": 5
                },
                "description": "各年齡層才藝展示"
            }
        ]
        
        self.scheduled_activities = weekly_activities
    
    def simulate_interaction(self, activity, current_time):
        """模擬跨齡互動"""
        zone = self.interaction_zones[activity["zone"]]
        participants = activity["participants"]
        
        # 生成參與者
        interaction_group = self.gather_participants(participants)
        
        # 移動到互動區域
        self.move_to_zone(interaction_group, zone["location"])
        
        # 執行互動動畫
        self.perform_interaction(
            interaction_group, 
            activity["name"],
            zone["activities"]
        )
        
        # 記錄互動數據
        self.log_interaction(activity, interaction_group)
    
    def gather_participants(self, participant_spec):
        """召集參與者"""
        participants = []
        
        for char_type, count in participant_spec.items():
            available_chars = self.get_available_characters(char_type)
            selected = random.sample(available_chars, min(count, len(available_chars)))
            participants.extend(selected)
        
        return participants
    
    def perform_interaction(self, participants, activity_name, zone_activities):
        """執行互動動畫"""
        if "gardening" in zone_activities:
            self.gardening_interaction(participants)
        elif "storytelling" in zone_activities:
            self.storytelling_interaction(participants)
        elif "shared_meals" in zone_activities:
            self.dining_interaction(participants)
        elif "performance" in zone_activities:
            self.performance_interaction(participants)
    
    def gardening_interaction(self, participants):
        """園藝互動動畫"""
        elderly = [p for p in participants if p["type"] == "elderly"]
        toddlers = [p for p in participants if p["type"] == "toddler"]
        
        for elder in elderly:
            # 長者示範動作
            self.play_animation(elder["path"], "teaching_gardening")
            
        for toddler in toddlers:
            # 幼兒模仿動作
            self.play_animation(toddler["path"], "learning_gardening")
    
    def storytelling_interaction(self, participants):
        """說故事互動"""
        storyteller = [p for p in participants if p["type"] == "elderly"][0]
        listeners = [p for p in participants if p["type"] == "toddler"]
        
        # 長者坐在椅子上
        self.play_animation(storyteller["path"], "sitting_storytelling")
        
        # 幼兒圍坐聆聽
        for listener in listeners:
            self.play_animation(listener["path"], "sitting_listening")
```

---

### 第六階段：感測器與數據收集（第11-12週）

#### 6.1 感測器配置
```python
# scripts/sensor_system.py
from omni.isaac.sensor import Camera, ContactSensor, IMUSensor
from omni.isaac.core.utils.prims import create_prim
import numpy as np

class SensorSystem:
    def __init__(self):
        self.cameras = {}
        self.contact_sensors = {}
        self.motion_sensors = {}
        self.setup_all_sensors()
    
    def setup_all_sensors(self):
        """配置全館感測器"""
        # 監視攝影機
        self.setup_cameras()
        
        # 接觸感測器（門、設備使用）
        self.setup_contact_sensors()
        
        # 動作感測器（跌倒偵測）
        self.setup_motion_sensors()
        
        # 環境感測器
        self.setup_environmental_sensors()
    
    def setup_cameras(self):
        """設定監視攝影機"""
        camera_positions = {
            # B1 停車場
            "b1_entrance": [-15, 0, -2],
            "b1_parking_1": [0, 0, -2],
            "b1_parking_2": [15, 0, -2],
            
            # 1F 長照中心
            "1f_entrance": [0, -10, 1.5],
            "1f_dementia_corridor": [-10, 0, 1.5],
            "1f_activity_room": [0, 0, 1.5],
            "1f_garden": [15, 10, 1.5],
            
            # 2F 托嬰中心
            "2f_infant_room": [-8, 0, 5],
            "2f_toddler_room": [5, 0, 5],
            "2f_outdoor_play": [12, 8, 5],
            
            # 3F 家庭支持
            "3f_multipurpose": [0, 0, 8.5],
            "3f_kitchen": [8, 0, 8.5],
            
            # 4F 青少年中心
            "4f_basketball": [0, 0, 12],
            "4f_study_room": [8, 5, 12]
        }
        
        for camera_name, position in camera_positions.items():
            camera_path = f"/World/Sensors/Cameras/{camera_name}"
            
            # 建立攝影機
            camera = Camera(
                prim_path=camera_path,
                frequency=30,  # 30 FPS
                resolution=(1920, 1080)
            )
            
            # 設定位置和角度
            camera.set_world_pose(position=position)
            
            # 設定 FOV
            camera.set_horizontal_aperture(20.955)
            camera.set_focal_length(18.147)
            
            # 啟用深度感測（用於跌倒偵測）
            camera.set_enable_depth_output(True)
            
            self.cameras[camera_name] = camera
    
    def setup_contact_sensors(self):
        """設定接觸感測器"""
        sensor_locations = [
            # 門禁感測
            ("/World/Building/1F/entrance_door", "door_sensor"),
            ("/World/Building/2F/entrance_door", "door_sensor"),
            
            # 設備使用感測
            ("/World/Building/1F/rehabilitation/equipment_1", "equipment_sensor"),
            ("/World/Building/4F/gym/treadmill", "equipment_sensor"),
            
            # 床位感測（午睡監測）
            ("/World/Building/2F/nap_room/bed_1", "bed_sensor"),
            ("/World/Building/1F/rest_area/recliner_1", "bed_sensor")
        ]
        
        for prim_path, sensor_type in sensor_locations:
            sensor_path = f"{prim_path}/contact_sensor"
            
            sensor = ContactSensor(
                prim_path=sensor_path,
                min_threshold=0.1,
                max_threshold=10000.0
            )
            
            self.contact_sensors[sensor_path] = {
                "sensor": sensor,
                "type": sensor_type
            }
    
    def setup_motion_sensors(self):
        """設定動作感測器（跌倒偵測）"""
        # 在關鍵區域設定 IMU 感測器
        critical_areas = [
            "1F/dementia_area",  # 失智專區
            "1F/bathroom",        # 浴室
            "2F/play_area",      # 遊戲區
            "4F/basketball"       # 籃球場
        ]
        
        for area in critical_areas:
            sensor_path = f"/World/Sensors/Motion/{area}"
            
            imu = IMUSensor(
                prim_path=sensor_path,
                frequency=100  # 100Hz 取樣率
            )
            
            self.motion_sensors[area] = imu
    
    def setup_environmental_sensors(self):
        """設定環境感測器"""
        # 溫濕度、CO2、噪音等
        env_sensors = {
            "temperature": {
                "range": (18, 28),
                "unit": "celsius"
            },
            "humidity": {
                "range": (40, 70),
                "unit": "percent"
            },
            "co2": {
                "range": (400, 1000),
                "unit": "ppm"
            },
            "noise": {
                "range": (30, 85),
                "unit": "decibel"
            }
        }
        
        # 每層樓配置環境感測器
        for floor in ["B1", "1F", "2F", "3F", "4F"]:
            for sensor_type, config in env_sensors.items():
                self.create_env_sensor(floor, sensor_type, config)
    
    def detect_fall(self, camera_name):
        """跌倒偵測算法"""
        camera = self.cameras[camera_name]
        
        # 取得深度影像
        depth_image = camera.get_current_frame()["depth"]
        
        # 簡單的跌倒偵測邏輯
        # 實際應用需要更複雜的 AI 模型
        height_threshold = 0.5  # 公尺
        
        # 偵測人物高度突然降低
        min_height = np.min(depth_image[depth_image > 0])
        
        if min_height < height_threshold:
            self.trigger_fall_alert(camera_name)
            return True
        
        return False
    
    def trigger_fall_alert(self, location):
        """觸發跌倒警報"""
        alert = {
            "type": "FALL_DETECTED",
            "location": location,
            "timestamp": omni.timeline.get_timeline_interface().get_current_time(),
            "priority": "HIGH"
        }
        
        # 發送警報到護理站
        self.send_alert_to_nurse_station(alert)
        
        # 記錄事件
        self.log_incident(alert)
    
    def monitor_occupancy(self):
        """監測空間使用率"""
        occupancy_data = {}
        
        for camera_name, camera in self.cameras.items():
            # 使用電腦視覺計算人數
            frame = camera.get_current_frame()["rgba"]
            
            # 這裡應該使用實際的人物偵測模型
            # 簡化示例
            people_count = self.count_people_in_frame(frame)
            
            floor = camera_name.split("_")[0].upper()
            if floor not in occupancy_data:
                occupancy_data[floor] = 0
            
            occupancy_data[floor] += people_count
        
        return occupancy_data
```

---

### 第七階段：AI 行為與緊急應變（第13-14週）

#### 7.1 AI 驅動的行為系統
```python
# scripts/ai_behavior_system.py
import numpy as np
from omni.isaac.core.utils.torch.rotations import *
import torch

class AIBehaviorSystem:
    def __init__(self):
        self.behavior_trees = {}
        self.emergency_protocols = {}
        self.load_ai_models()
        self.setup_behaviors()
    
    def load_ai_models(self):
        """載入預訓練的 AI 模型"""
        self.models = {
            "fall_detection": self.load_fall_detection_model(),
            "behavior_prediction": self.load_behavior_model(),
            "crowd_analysis": self.load_crowd_model(),
            "emotion_recognition": self.load_emotion_model()
        }
    
    def setup_behaviors(self):
        """設定 AI 驅動的行為"""
        # 長者行為模式
        self.behavior_trees["elderly"] = {
            "normal": {
                "morning_routine": ["arrival", "health_check", "exercise"],
                "afternoon_routine": ["lunch", "rest", "activity"],
                "evening_routine": ["snack", "social", "departure"]
            },
            "dementia": {
                "wandering": ["walk_random", "check_exit", "return"],
                "sundowning": ["agitation", "confusion", "calming"]
            }
        }
        
        # 幼兒行為模式
        self.behavior_trees["toddler"] = {
            "active": ["explore", "play", "interact"],
            "tired": ["fussy", "seek_comfort", "sleep"],
            "hungry": ["cry", "seek_food", "eat"]
        }
        
        # 緊急應變協議
        self.emergency_protocols = {
            "fire": self.fire_evacuation_protocol,
            "earthquake": self.earthquake_protocol,
            "medical": self.medical_emergency_protocol,
            "missing_person": self.missing_person_protocol
        }
    
    def predict_behavior(self, character_data):
        """預測角色下一步行為"""
        model = self.models["behavior_prediction"]
        
        # 準備輸入特徵
        features = self.extract_features(character_data)
        
        # 預測
        with torch.no_grad():
            prediction = model(features)
        
        return self.decode_behavior(prediction)
    
    def fire_evacuation_protocol(self):
        """火災疏散協議"""
        evacuation_plan = {
            "4F": {
                "primary_route": "main_stairs_west",
                "secondary_route": "emergency_stairs_east",
                "assembly_point": "outdoor_parking"
            },
            "3F": {
                "primary_route": "main_stairs_west",
                "secondary_route": "emergency_stairs_east",
                "assembly_point": "outdoor_parking"
            },
            "2F": {
                "primary_route": "main_stairs_west",
                "secondary_route": "emergency_stairs_east",
                "special_procedure": "carry_infants",
                "equipment": ["evacuation_cribs", "baby_carriers"],
                "assembly_point": "garden"
            },
            "1F": {
                "primary_route": "main_entrance",
                "secondary_route": "garden_exit",
                "special_procedure": "wheelchair_priority",
                "assembly_point": "garden"
            }
        }
        
        # 執行疏散
        for floor, plan in evacuation_plan.items():
            self.execute_evacuation(floor, plan)
    
    def execute_evacuation(self, floor, plan):
        """執行疏散程序"""
        # 觸發警報
        self.trigger_alarm(floor)
        
        # 解鎖所有緊急出口
        self.unlock_emergency_exits(floor)
        
        # 引導人員疏散
        characters_on_floor = self.get_characters_on_floor(floor)
        
        for character in characters_on_floor:
            if "special_procedure" in plan:
                if plan["special_procedure"] == "carry_infants":
                    self.evacuate_with_infant(character)
                elif plan["special_procedure"] == "wheelchair_priority":
                    self.evacuate_with_wheelchair(character)
            else:
                self.evacuate_normal(character, plan["primary_route"])
    
    def detect_abnormal_behavior(self, character_path):
        """偵測異常行為"""
        character = self.get_character_data(character_path)
        
        abnormal_patterns = {
            "fall": self.detect_fall_pattern,
            "wandering": self.detect_wandering,
            "distress": self.detect_distress,
            "aggression": self.detect_aggression
        }
        
        for pattern_name, detector in abnormal_patterns.items():
            if detector(character):
                self.handle_abnormal_behavior(character_path, pattern_name)
    
    def detect_wandering(self, character):
        """偵測徘徊行為（失智症狀）"""
        # 檢查移動模式
        movement_history = character.get("movement_history", [])
        
        if len(movement_history) < 10:
            return False
        
        # 計算路徑的重複性
        positions = [m["position"] for m in movement_history[-10:]]
        
        # 檢查是否在小範圍內重複移動
        distances = []
        for i in range(1, len(positions)):
            dist = np.linalg.norm(
                np.array(positions[i]) - np.array(positions[i-1])
            )
            distances.append(dist)
        
        # 如果平均移動距離很小但總移動次數很多
        avg_distance = np.mean(distances)
        total_distance = np.sum(distances)
        
        if avg_distance < 2.0 and total_distance > 20.0:
            return True
        
        return False
    
    def crowd_dynamics_simulation(self, zone):
        """群眾動力學模擬"""
        crowd_model = self.models["crowd_analysis"]
        
        # 取得區域內所有人員
        people_in_zone = self.get_people_in_zone(zone)
        
        # 計算群眾密度
        density = len(people_in_zone) / zone["area"]
        
        # 預測群眾流動
        flow_prediction = crowd_model.predict({
            "density": density,
            "exits": zone["exits"],
            "obstacles": zone["obstacles"]
        })
        
        # 調整人員移動以避免擁擠
        if density > 0.5:  # 每2平方公尺超過1人
            self.redistribute_crowd(people_in_zone, flow_prediction)
```

---

### 第八階段：視覺化與分析工具（第15-16週）

#### 8.1 即時監控儀表板
```python
# scripts/visualization_dashboard.py
from omni.ui import workspace, window
import omni.ui as ui
import carb

class MonitoringDashboard:
    def __init__(self):
        self.window = None
        self.charts = {}
        self.create_dashboard()
    
    def create_dashboard(self):
        """建立監控儀表板"""
        self.window = ui.Window(
            "赤土崎社福設施 - 即時監控",
            width=1200,
            height=800
        )
        
        with self.window.frame:
            with ui.VStack():
                # 標題
                ui.Label(
                    "即時監控系統",
                    style={"font_size": 24, "color": 0xFFFFFFFF}
                )
                
                # 主要監控區域
                with ui.HStack(height=600):
                    # 左側 - 樓層狀態
                    self.create_floor_status_panel()
                    
                    # 中間 - 即時影像
                    self.create_camera_feeds()
                    
                    # 右側 - 警報與通知
                    self.create_alert_panel()
                
                # 底部 - 統計資訊
                self.create_statistics_panel()
    
    def create_floor_status_panel(self):
        """樓層狀態面板"""
        with ui.VStack(width=300):
            ui.Label("樓層使用狀態", style={"font_size": 18})
            
            floors = ["B1", "1F", "2F", "3F", "4F"]
            self.floor_indicators = {}
            
            for floor in floors:
                with ui.HStack():
                    ui.Label(f"{floor}:")
                    
                    # 人數指示
                    occupancy = ui.Label("0/50")
                    self.floor_indicators[f"{floor}_occupancy"] = occupancy
                    
                    # 狀態指示燈
                    status_color = ui.ColorWidget(0.0, 1.0, 0.0)
                    self.floor_indicators[f"{floor}_status"] = status_color
    
    def create_camera_feeds(self):
        """攝影機畫面"""
        with ui.VStack(width=600):
            ui.Label("即時監控畫面", style={"font_size": 18})
            
            # 建立 2x2 的攝影機格子
            with ui.HStack():
                with ui.VStack():
                    # 攝影機 1 - 1F 失智區
                    self.camera_view_1 = ui.ImageWithProvider(
                        width=280, height=200
                    )
                    ui.Label("1F 失智專區")
                    
                    # 攝影機 2 - 2F 遊戲區
                    self.camera_view_2 = ui.ImageWithProvider(
                        width=280, height=200
                    )
                    ui.Label("2F 遊戲區")
                
                with ui.VStack():
                    # 攝影機 3 - 4F 籃球場
                    self.camera_view_3 = ui.ImageWithProvider(
                        width=280, height=200
                    )
                    ui.Label("4F 籃球場")
                    
                    # 攝影機 4 - B1 停車場
                    self.camera_view_4 = ui.ImageWithProvider(
                        width=280, height=200
                    )
                    ui.Label("B1 停車場")
    
    def create_alert_panel(self):
        """警報面板"""
        with ui.VStack(width=300):
            ui.Label("警報與通知", style={"font_size": 18})
            
            # 警報列表
            self.alert_list = ui.ScrollingFrame(height=400)
            
            with self.alert_list:
                with ui.VStack():
                    self.alerts = []
                    # 預設顯示"無警報"
                    self.no_alert_label = ui.Label(
                        "系統正常運作中",
                        style={"color": 0xFF00FF00}
                    )
    
    def create_statistics_panel(self):
        """統計資訊面板"""
        with ui.HStack(height=150):
            # 今日統計
            with ui.VStack():
                ui.Label("今日統計")
                ui.Label("總入場人數: 156")
                ui.Label("跨齡活動: 3 場")
                ui.Label("緊急事件: 0")
            
            # 設備使用率
            with ui.VStack():
                ui.Label("設備使用率")
                ui.Label("復健設備: 78%")
                ui.Label("活動空間: 65%")
                ui.Label("停車位: 23/30")
            
            # 人員配置
            with ui.VStack():
                ui.Label("人員配置")
                ui.Label("照護人員: 12/12")
                ui.Label("行政人員: 3/3")
                ui.Label("志工: 5")
    
    def update_dashboard(self, data):
        """更新儀表板數據"""
        # 更新樓層狀態
        for floor, info in data["floors"].items():
            occupancy_label = self.floor_indicators[f"{floor}_occupancy"]
            occupancy_label.text = f"{info['current']}/{info['capacity']}"
            
            # 更新狀態燈顏色
            status = self.floor_indicators[f"{floor}_status"]
            if info['current'] > info['capacity'] * 0.9:
                status.set_rgb(1.0, 0.0, 0.0)  # 紅色 - 接近滿載
            elif info['current'] > info['capacity'] * 0.7:
                status.set_rgb(1.0, 1.0, 0.0)  # 黃色 - 中等
            else:
                status.set_rgb(0.0, 1.0, 0.0)  # 綠色 - 正常
        
        # 更新警報
        if "alerts" in data:
            for alert in data["alerts"]:
                self.add_alert(alert)
    
    def add_alert(self, alert_data):
        """新增警報"""
        with self.alert_list:
            with ui.VStack():
                # 移除"無警報"標籤
                if self.no_alert_label:
                    self.no_alert_label.visible = False
                
                # 新增警報項目
                alert_color = 0xFFFF0000 if alert_data["priority"] == "HIGH" else 0xFFFFFF00
                
                with ui.HStack():
                    ui.Label(
                        f"[{alert_data['time']}]",
                        style={"color": alert_color}
                    )
                    ui.Label(
                        alert_data["message"],
                        style={"color": alert_color}
                    )
```

#### 8.2 數據分析與報告生成
```python
# scripts/analytics_reporting.py
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime, timedelta

class AnalyticsReporting:
    def __init__(self):
        self.data_storage = []
        self.reports = {}
        
    def collect_simulation_data(self, timestamp):
        """收集模擬數據"""
        data_point = {
            "timestamp": timestamp,
            "floor_occupancy": self.get_floor_occupancy(),
            "activity_participation": self.get_activity_participation(),
            "equipment_usage": self.get_equipment_usage(),
            "incidents": self.get_incidents(),
            "interactions": self.get_interaction_data()
        }
        
        self.data_storage.append(data_point)
    
    def generate_daily_report(self):
        """生成每日報告"""
        df = pd.DataFrame(self.data_storage)
        
        report = {
            "date": datetime.now().strftime("%Y-%m-%d"),
            "summary": {},
            "details": {},
            "recommendations": []
        }
        
        # 摘要統計
        report["summary"] = {
            "total_visitors": self.calculate_total_visitors(df),
            "peak_hour": self.find_peak_hour(df),
            "avg_stay_duration": self.calculate_avg_duration(df),
            "incident_count": len(df["incidents"].sum()),
            "interaction_events": len(df["interactions"].sum())
        }
        
        # 詳細分析
        report["details"] = {
            "floor_utilization": self.analyze_floor_utilization(df),
            "activity_success": self.analyze_activity_success(df),
            "safety_metrics": self.analyze_safety_metrics(df),
            "cross_generation": self.analyze_interactions(df)
        }
        
        # 生成建議
        report["recommendations"] = self.generate_recommendations(report)
        
        return report
    
    def visualize_data(self, data_type="occupancy"):
        """視覺化數據"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        
        # 樓層使用率趨勢
        self.plot_occupancy_trend(axes[0, 0])
        
        # 跨齡互動熱圖
        self.plot_interaction_heatmap(axes[0, 1])
        
        # 設備使用統計
        self.plot_equipment_usage(axes[1, 0])
        
        # 安全事件分布
        self.plot_incident_distribution(axes[1, 1])
        
        plt.tight_layout()
        return fig
    
    def export_report(self, format="pdf"):
        """匯出報告"""
        report = self.generate_daily_report()
        
        if format == "pdf":
            self.export_pdf_report(report)
        elif format == "excel":
            self.export_excel_report(report)
        elif format == "json":
            self.export_json_report(report)
```

---

## 🚀 執行步驟總結

### 快速開始指令
```bash
# 1. 複製專案
git clone https://github.com/yourusername/easterlin-hsinchu-3d.git
cd easterlin-hsinchu-3d

# 2. 安裝相依套件
pip install -r requirements.txt

# 3. 啟動 Isaac Sim
./isaac-sim.sh

# 4. 執行主程式
python main.py --config configs/building_config.yaml
```

### 主程式架構
```python
# main.py
import omni
from scripts.building_generator import BuildingGenerator
from scripts.facility_placer import FacilityPlacer
from scripts.character_simulation import CharacterSimulation
from scripts.sensor_system import SensorSystem
from scripts.ai_behavior_system import AIBehaviorSystem
from scripts.visualization_dashboard import MonitoringDashboard

def main():
    # 初始化 Isaac Sim
    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({"headless": False})
    
    # 建立建築物
    print("建立建築結構...")
    building = BuildingGenerator()
    building.generate_building()
    
    # 配置設施
    print("配置設施...")
    facilities = FacilityPlacer("configs/floor_configs.yaml")
    facilities.setup_all_facilities()
    
    # 設定人物
    print("生成人物...")
    characters = CharacterSimulation()
    
    # 設定感測器
    print("配置感測器...")
    sensors = SensorSystem()
    
    # 啟動 AI 系統
    print("啟動 AI 系統...")
    ai_system = AIBehaviorSystem()
    
    # 開啟監控面板
    print("開啟監控面板...")
    dashboard = MonitoringDashboard()
    
    # 開始模擬
    print("開始模擬...")
    simulation_app.update()
    
    # 主要模擬迴圈
    while simulation_app.is_running():
        # 更新模擬
        simulation_app.update()
        
        # 更新 AI 行為
        ai_system.update_all_behaviors()
        
        # 更新儀表板
        dashboard.update_dashboard(sensors.get_current_data())
    
    simulation_app.close()

if __name__ == "__main__":
    main()
```

---

## 📊 預期成果

### 完成後您將擁有：
1. **完整的 3D 建築模型** - 包含所有樓層和空間配置
2. **動態人物模擬** - 140-180個AI驅動的虛擬角色
3. **智慧感測系統** - 即時監控和數據收集
4. **跨齡互動場景** - 可視化的代間活動
5. **緊急應變演練** - 火災、地震等情境模擬
6. **數據分析報告** - 空間使用效率、安全指標等

### 應用價值：
- **設計驗證** - 在實際建造前測試設計合理性
- **營運規劃** - 優化人員配置和動線設計  
- **安全演練** - 模擬各種緊急情況的應對
- **培訓工具** - 新進人員的虛擬培訓環境
- **決策支援** - 基於數據的營運改善建議

---

## 💡 進階優化建議

1. **整合 BIM 模型** - 如果有 Revit/ArchiCAD 模型可直接匯入
2. **VR/AR 體驗** - 加入頭戴裝置支援，提供沉浸式體驗
3. **雲端部署** - 使用 NVIDIA Omniverse Cloud 進行協作
4. **AI 模型訓練** - 使用模擬數據訓練更精確的行為預測模型
5. **IoT 整合** - 連接實際感測器數據進行數位孿生

---

祝您專案實作順利！如有任何問題，歡迎隨時詢問。
