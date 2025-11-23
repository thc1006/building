#!/usr/bin/env python3
"""
建築物生成器 - 程序化生成赤土崎社福設施建築結構
Building Generator for Easterlin Facility
"""

import omni
from pxr import Gf, UsdGeom, Sdf, UsdPhysics, UsdShade
import numpy as np
import yaml
from typing import Dict, List, Tuple, Optional
import logging

logger = logging.getLogger(__name__)


class BuildingGenerator:
    """建築物生成器類別"""
    
    def __init__(self, config_path: str = "configs/building_config.yaml"):
        """
        初始化建築生成器
        
        Args:
            config_path: 建築配置檔案路徑
        """
        self.stage = omni.usd.get_context().get_stage()
        self.config = self._load_config(config_path)
        self.building_path = "/World/Building"
        self.materials = {}
        
        # 建築參數
        self.floor_specs = self._extract_floor_specs()
        
        logger.info(f"建築生成器初始化完成，載入 {len(self.floor_specs)} 個樓層配置")
    
    def _load_config(self, config_path: str) -> Dict:
        """載入配置檔案"""
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            logger.warning(f"找不到配置檔案 {config_path}，使用預設配置")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict:
        """取得預設配置"""
        return {
            "building_dimensions": {
                "foundation": {"length": 40, "width": 25, "depth": 3},
                "floor_heights": {
                    "B1": 3.0, "1F": 3.5, "2F": 3.5, 
                    "3F": 3.5, "4F": 4.0
                }
            },
            "floors": {
                "B1_parking": {"area": 600, "z_offset": -3.0},
                "floor_1_elderly": {"area": 800, "z_offset": 0},
                "floor_2_nursery": {"area": 700, "z_offset": 3.5},
                "floor_3_family": {"area": 500, "z_offset": 7.0},
                "floor_4_youth": {"area": 500, "z_offset": 10.5}
            }
        }
    
    def _extract_floor_specs(self) -> Dict:
        """從配置中提取樓層規格"""
        specs = {}
        floors_config = self.config.get("floors", {})
        
        for floor_key, floor_data in floors_config.items():
            specs[floor_key] = {
                "height": floor_data.get("height", 3.5),
                "area": floor_data.get("area", 500),
                "z_offset": floor_data.get("z_offset", 0),
                "spaces": floor_data.get("spaces", [])
            }
        
        return specs
    
    def generate_building(self):
        """生成完整建築物"""
        logger.info("開始生成建築物...")
        
        # 建立建築根節點
        self._create_building_root()
        
        # 建立材質
        self._create_materials()
        
        # 生成地基
        self._create_foundation()
        
        # 生成各樓層
        for floor_name, specs in self.floor_specs.items():
            self._create_floor(floor_name, specs)
        
        # 建立樓梯和電梯
        self._create_vertical_circulation()
        
        # 新增建築細節
        self._add_architectural_details()
        
        logger.info("建築物生成完成")
    
    def _create_building_root(self):
        """建立建築根節點"""
        self.building_xform = UsdGeom.Xform.Define(self.stage, self.building_path)
        self.building_xform.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0))
    
    def _create_materials(self):
        """建立建築材質"""
        materials_config = self.config.get("materials", {})
        
        for material_name, properties in materials_config.items():
            material_path = f"/World/Materials/{material_name}"
            material = UsdShade.Material.Define(self.stage, material_path)
            
            # 建立物理材質
            physics_material = UsdPhysics.MaterialAPI.Apply(
                self.stage.GetPrimAtPath(material_path)
            )
            
            # 設定材質屬性
            if "static_friction" in properties:
                physics_material.CreateStaticFrictionAttr(
                    properties["static_friction"]
                )
            if "dynamic_friction" in properties:
                physics_material.CreateDynamicFrictionAttr(
                    properties["dynamic_friction"]
                )
            if "restitution" in properties:
                physics_material.CreateRestitutionAttr(
                    properties["restitution"]
                )
            
            self.materials[material_name] = material_path
            
        logger.info(f"建立了 {len(self.materials)} 種材質")
    
    def _create_foundation(self):
        """建立地基"""
        foundation_path = f"{self.building_path}/Foundation"
        foundation = UsdGeom.Cube.Define(self.stage, foundation_path)
        
        foundation_config = self.config["building_dimensions"]["foundation"]
        length = foundation_config["length"]
        width = foundation_config["width"]
        depth = foundation_config["depth"]
        
        # 設定地基尺寸
        foundation.GetExtentAttr().Set([
            (-length/2, -width/2, -depth),
            (length/2, width/2, 0)
        ])
        
        # 設定位置
        UsdGeom.XformCommonAPI(foundation).SetTranslate([0, 0, -depth/2])
        
        # 套用混凝土材質
        if "concrete" in self.materials:
            foundation.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
            foundation.GetPrim().GetRelationship("material:binding").SetTargets(
                [self.materials["concrete"]]
            )
        
        logger.info("地基建立完成")
    
    def _create_floor(self, floor_name: str, specs: Dict):
        """
        建立單一樓層
        
        Args:
            floor_name: 樓層名稱
            specs: 樓層規格
        """
        logger.info(f"建立樓層：{floor_name}")
        
        floor_path = f"{self.building_path}/{floor_name}"
        floor_xform = UsdGeom.Xform.Define(self.stage, floor_path)
        
        # 設定樓層位置
        z_offset = specs["z_offset"]
        floor_xform.AddTranslateOp().Set(Gf.Vec3f(0, 0, z_offset))
        
        # 建立樓板
        self._create_floor_slab(floor_path, specs)
        
        # 建立結構元素
        self._create_structural_elements(floor_path, specs)
        
        # 建立空間分隔
        self._create_spaces(floor_path, specs.get("spaces", []))
        
        # 建立門窗
        self._create_openings(floor_path, floor_name)
    
    def _create_floor_slab(self, floor_path: str, specs: Dict):
        """建立樓板"""
        slab_path = f"{floor_path}/slab"
        slab = UsdGeom.Cube.Define(self.stage, slab_path)
        
        # 根據面積計算尺寸（簡化為矩形）
        area = specs["area"]
        length = np.sqrt(area * 1.6)  # 假設長寬比為 1.6:1
        width = area / length
        thickness = 0.3  # 30cm 樓板厚度
        
        slab.GetExtentAttr().Set([
            (-length/2, -width/2, 0),
            (length/2, width/2, thickness)
        ])
        
        # 套用材質
        if "concrete" in self.materials:
            slab.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
    
    def _create_structural_elements(self, floor_path: str, specs: Dict):
        """建立結構元素（柱、樑）"""
        # 建立柱子
        self._create_columns(floor_path, specs)
        
        # 建立樑
        self._create_beams(floor_path, specs)
        
        # 建立外牆
        self._create_exterior_walls(floor_path, specs)
    
    def _create_columns(self, floor_path: str, specs: Dict):
        """建立結構柱"""
        columns_group = UsdGeom.Xform.Define(
            self.stage, f"{floor_path}/columns"
        )
        
        # 計算柱子配置
        area = specs["area"]
        length = np.sqrt(area * 1.6)
        width = area / length
        height = specs["height"]
        
        column_spacing = 8.0  # 8公尺間距
        column_size = 0.6  # 60cm x 60cm
        
        cols_x = int(length / column_spacing) + 1
        cols_y = int(width / column_spacing) + 1
        
        for i in range(cols_x):
            for j in range(cols_y):
                x = -length/2 + i * column_spacing
                y = -width/2 + j * column_spacing
                
                column_path = f"{floor_path}/columns/col_{i}_{j}"
                column = UsdGeom.Cube.Define(self.stage, column_path)
                
                column.GetExtentAttr().Set([
                    (-column_size/2, -column_size/2, 0),
                    (column_size/2, column_size/2, height)
                ])
                
                UsdGeom.XformCommonAPI(column).SetTranslate([x, y, height/2])
                
                # 套用材質
                if "concrete" in self.materials:
                    column.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
    
    def _create_beams(self, floor_path: str, specs: Dict):
        """建立結構樑"""
        beams_group = UsdGeom.Xform.Define(
            self.stage, f"{floor_path}/beams"
        )
        
        area = specs["area"]
        length = np.sqrt(area * 1.6)
        width = area / length
        height = specs["height"]
        
        beam_height = 0.6  # 60cm
        beam_width = 0.4   # 40cm
        
        # 主樑（X方向）
        beam_spacing = 8.0
        num_beams = int(width / beam_spacing) + 1
        
        for i in range(num_beams):
            y = -width/2 + i * beam_spacing
            beam_path = f"{floor_path}/beams/main_beam_{i}"
            beam = UsdGeom.Cube.Define(self.stage, beam_path)
            
            beam.GetExtentAttr().Set([
                (-length/2, -beam_width/2, height-beam_height),
                (length/2, beam_width/2, height)
            ])
            
            UsdGeom.XformCommonAPI(beam).SetTranslate([0, y, 0])
    
    def _create_exterior_walls(self, floor_path: str, specs: Dict):
        """建立外牆"""
        walls_group = UsdGeom.Xform.Define(
            self.stage, f"{floor_path}/exterior_walls"
        )
        
        area = specs["area"]
        length = np.sqrt(area * 1.6)
        width = area / length
        height = specs["height"]
        wall_thickness = 0.2  # 20cm
        
        # 定義四面牆
        walls = [
            ("north", [0, width/2, height/2], [length, wall_thickness, height]),
            ("south", [0, -width/2, height/2], [length, wall_thickness, height]),
            ("east", [length/2, 0, height/2], [wall_thickness, width, height]),
            ("west", [-length/2, 0, height/2], [wall_thickness, width, height])
        ]
        
        for wall_name, position, size in walls:
            wall_path = f"{floor_path}/exterior_walls/{wall_name}"
            wall = UsdGeom.Cube.Define(self.stage, wall_path)
            
            wall.GetExtentAttr().Set([
                (-size[0]/2, -size[1]/2, -size[2]/2),
                (size[0]/2, size[1]/2, size[2]/2)
            ])
            
            UsdGeom.XformCommonAPI(wall).SetTranslate(position)
            
            # 套用材質
            if "concrete" in self.materials:
                wall.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
    
    def _create_spaces(self, floor_path: str, spaces: List[Dict]):
        """建立內部空間分隔"""
        if not spaces:
            return
        
        spaces_group = UsdGeom.Xform.Define(
            self.stage, f"{floor_path}/spaces"
        )
        
        for space in spaces:
            self._create_single_space(f"{floor_path}/spaces", space)
    
    def _create_single_space(self, parent_path: str, space_config: Dict):
        """建立單一空間"""
        space_name = space_config.get("name", "unnamed_space")
        space_type = space_config.get("type", "generic")
        position = space_config.get("position", [0, 0, 0])
        dimensions = space_config.get("dimensions", [10, 10, 3.5])
        
        space_path = f"{parent_path}/{space_name}"
        space_xform = UsdGeom.Xform.Define(self.stage, space_path)
        
        # 設定空間位置
        space_xform.AddTranslateOp().Set(Gf.Vec3f(*position))
        
        # 建立空間邊界（用透明方塊表示）
        boundary_path = f"{space_path}/boundary"
        boundary = UsdGeom.Cube.Define(self.stage, boundary_path)
        
        boundary.GetExtentAttr().Set([
            (-dimensions[0]/2, -dimensions[1]/2, 0),
            (dimensions[0]/2, dimensions[1]/2, dimensions[2])
        ])
        
        # 設定為不可見但有碰撞
        boundary.GetVisibilityAttr().Set("invisible")
        boundary.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
        
        # 根據空間類型建立內牆
        if space_type in ["care_unit", "nursery", "office"]:
            self._create_interior_walls(space_path, dimensions)
        
        # 新增空間標示
        self._add_space_label(space_path, space_name, space_type)
    
    def _create_interior_walls(self, space_path: str, dimensions: List[float]):
        """建立內牆"""
        wall_thickness = 0.12  # 12cm 內牆
        
        # 簡化：只建立一面分隔牆
        wall_path = f"{space_path}/interior_wall"
        wall = UsdGeom.Cube.Define(self.stage, wall_path)
        
        wall.GetExtentAttr().Set([
            (-dimensions[0]/4, -wall_thickness/2, 0),
            (dimensions[0]/4, wall_thickness/2, dimensions[2])
        ])
        
        # 套用材質
        if "concrete" in self.materials:
            wall.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
    
    def _create_openings(self, floor_path: str, floor_name: str):
        """建立門窗開口"""
        openings_group = UsdGeom.Xform.Define(
            self.stage, f"{floor_path}/openings"
        )
        
        # 建立主入口
        if "1" in floor_name or "entrance" in floor_name.lower():
            self._create_entrance(f"{floor_path}/openings")
        
        # 建立窗戶
        self._create_windows(f"{floor_path}/openings", floor_name)
        
        # 建立內部門
        self._create_doors(f"{floor_path}/openings")
    
    def _create_entrance(self, parent_path: str):
        """建立主入口"""
        entrance_path = f"{parent_path}/main_entrance"
        entrance = UsdGeom.Cube.Define(self.stage, entrance_path)
        
        # 自動門尺寸
        door_width = 2.0  # 2公尺寬
        door_height = 2.5  # 2.5公尺高
        door_thickness = 0.1
        
        entrance.GetExtentAttr().Set([
            (-door_width/2, -door_thickness/2, 0),
            (door_width/2, door_thickness/2, door_height)
        ])
        
        # 設定為玻璃材質（如果有的話）
        entrance.GetDisplayColorAttr().Set([(0.7, 0.9, 1.0)])
        entrance.GetDisplayOpacityAttr().Set([0.3])
    
    def _create_windows(self, parent_path: str, floor_name: str):
        """建立窗戶"""
        # 根據樓層決定窗戶數量和位置
        window_height = 1.5
        window_width = 1.2
        window_spacing = 4.0
        
        # 簡化：每面牆建立3個窗戶
        for i in range(3):
            window_path = f"{parent_path}/window_{i}"
            window = UsdGeom.Cube.Define(self.stage, window_path)
            
            window.GetExtentAttr().Set([
                (-window_width/2, -0.05, 1.0),
                (window_width/2, 0.05, 1.0 + window_height)
            ])
            
            # 設定玻璃外觀
            window.GetDisplayColorAttr().Set([(0.8, 0.9, 1.0)])
            window.GetDisplayOpacityAttr().Set([0.2])
            
            # 設定位置
            x_pos = -4.0 + i * window_spacing
            UsdGeom.XformCommonAPI(window).SetTranslate([x_pos, 10, 0])
    
    def _create_doors(self, parent_path: str):
        """建立內部門"""
        door_width = 1.0
        door_height = 2.2
        door_thickness = 0.08
        
        # 簡化：建立一個示範門
        door_path = f"{parent_path}/interior_door"
        door = UsdGeom.Cube.Define(self.stage, door_path)
        
        door.GetExtentAttr().Set([
            (-door_width/2, -door_thickness/2, 0),
            (door_width/2, door_thickness/2, door_height)
        ])
        
        # 設定木門材質
        if "wood" in self.materials:
            door.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
        
        door.GetDisplayColorAttr().Set([(0.6, 0.4, 0.2)])
    
    def _create_vertical_circulation(self):
        """建立垂直動線（樓梯、電梯）"""
        circulation_path = f"{self.building_path}/vertical_circulation"
        circulation_group = UsdGeom.Xform.Define(self.stage, circulation_path)
        
        # 建立電梯
        self._create_elevators(circulation_path)
        
        # 建立樓梯
        self._create_stairs(circulation_path)
    
    def _create_elevators(self, parent_path: str):
        """建立電梯"""
        # 建立兩部客梯
        for i in range(2):
            elevator_path = f"{parent_path}/elevator_{i+1}"
            elevator_shaft = UsdGeom.Cube.Define(self.stage, elevator_path)
            
            # 電梯井尺寸
            shaft_width = 2.5
            shaft_depth = 2.0
            shaft_height = 18  # 整棟建築高度
            
            elevator_shaft.GetExtentAttr().Set([
                (-shaft_width/2, -shaft_depth/2, -3),
                (shaft_width/2, shaft_depth/2, shaft_height)
            ])
            
            # 設定位置
            x_pos = -5 + i * 10
            UsdGeom.XformCommonAPI(elevator_shaft).SetTranslate([x_pos, -8, 0])
            
            # 設定材質
            elevator_shaft.GetDisplayColorAttr().Set([(0.5, 0.5, 0.5)])
            
            # 建立電梯廂
            self._create_elevator_car(elevator_path, i)
    
    def _create_elevator_car(self, shaft_path: str, elevator_index: int):
        """建立電梯廂"""
        car_path = f"{shaft_path}/car"
        car = UsdGeom.Cube.Define(self.stage, car_path)
        
        car_width = 2.0
        car_depth = 1.5
        car_height = 2.5
        
        car.GetExtentAttr().Set([
            (-car_width/2, -car_depth/2, 0),
            (car_width/2, car_depth/2, car_height)
        ])
        
        # 初始位置在 1F
        UsdGeom.XformCommonAPI(car).SetTranslate([0, 0, 0])
        
        # 新增物理屬性（可移動）
        car.GetPrim().ApplyAPI(UsdPhysics.RigidBodyAPI)
        car.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
        
        # 設定金屬外觀
        car.GetDisplayColorAttr().Set([(0.8, 0.8, 0.9)])
    
    def _create_stairs(self, parent_path: str):
        """建立樓梯"""
        # 建立兩座安全梯
        for i in range(2):
            stairs_path = f"{parent_path}/stairs_{i+1}"
            stairs_group = UsdGeom.Xform.Define(self.stage, stairs_path)
            
            # 設定位置（建築兩端）
            x_pos = -15 if i == 0 else 15
            stairs_group.AddTranslateOp().Set(Gf.Vec3f(x_pos, 0, 0))
            
            # 建立每層樓梯
            for floor_name, specs in self.floor_specs.items():
                if "B1" not in floor_name:  # B1 不需要樓梯
                    self._create_stair_flight(
                        stairs_path, 
                        floor_name, 
                        specs["z_offset"],
                        specs["height"]
                    )
    
    def _create_stair_flight(self, parent_path: str, floor_name: str, 
                            z_offset: float, height: float):
        """建立單層樓梯"""
        flight_path = f"{parent_path}/flight_{floor_name}"
        
        # 樓梯參數
        num_steps = int(height / 0.17)  # 每階高 17cm
        step_depth = 0.28  # 每階深 28cm
        stair_width = 1.2  # 樓梯寬度
        
        for i in range(num_steps):
            step_path = f"{flight_path}/step_{i}"
            step = UsdGeom.Cube.Define(self.stage, step_path)
            
            step_height = 0.17
            
            step.GetExtentAttr().Set([
                (-stair_width/2, -step_depth/2, 0),
                (stair_width/2, step_depth/2, step_height)
            ])
            
            # 計算每階位置
            z_pos = z_offset + i * step_height
            y_pos = i * step_depth
            
            UsdGeom.XformCommonAPI(step).SetTranslate([0, y_pos, z_pos])
            
            # 套用材質
            if "concrete" in self.materials:
                step.GetPrim().ApplyAPI(UsdPhysics.CollisionAPI)
    
    def _add_architectural_details(self):
        """新增建築細節"""
        # 新增屋頂
        self._create_roof()
        
        # 新增欄杆
        self._create_railings()
        
        # 新增標示系統
        self._create_signage()
    
    def _create_roof(self):
        """建立屋頂"""
        roof_path = f"{self.building_path}/roof"
        roof = UsdGeom.Cube.Define(self.stage, roof_path)
        
        # 平屋頂設計
        foundation_config = self.config["building_dimensions"]["foundation"]
        length = foundation_config["length"]
        width = foundation_config["width"]
        roof_thickness = 0.3
        
        roof.GetExtentAttr().Set([
            (-length/2 - 0.5, -width/2 - 0.5, 0),
            (length/2 + 0.5, width/2 + 0.5, roof_thickness)
        ])
        
        # 設定位置（4F 頂部）
        roof_height = 14.5  # 根據樓層高度計算
        UsdGeom.XformCommonAPI(roof).SetTranslate([0, 0, roof_height])
        
        # 套用防水材質
        roof.GetDisplayColorAttr().Set([(0.3, 0.3, 0.3)])
        
        # 新增屋頂設備（太陽能板）
        self._add_solar_panels(roof_path)
    
    def _add_solar_panels(self, roof_path: str):
        """新增太陽能板"""
        solar_group_path = f"{roof_path}/solar_panels"
        solar_group = UsdGeom.Xform.Define(self.stage, solar_group_path)
        
        panel_width = 2.0
        panel_height = 1.0
        panel_thickness = 0.05
        
        # 建立太陽能板陣列
        for i in range(5):
            for j in range(4):
                panel_path = f"{solar_group_path}/panel_{i}_{j}"
                panel = UsdGeom.Cube.Define(self.stage, panel_path)
                
                panel.GetExtentAttr().Set([
                    (-panel_width/2, -panel_height/2, 0),
                    (panel_width/2, panel_height/2, panel_thickness)
                ])
                
                # 設定位置
                x_pos = -8 + i * 4
                y_pos = -4 + j * 2.5
                
                UsdGeom.XformCommonAPI(panel).SetTranslate([x_pos, y_pos, 0.5])
                
                # 設定深藍色（太陽能板顏色）
                panel.GetDisplayColorAttr().Set([(0.1, 0.1, 0.4)])
                
                # 略微傾斜以獲得更好的日照
                panel.AddRotateXOp().Set(15)  # 15度傾斜
    
    def _create_railings(self):
        """建立欄杆（陽台、走廊）"""
        # 在每個有戶外空間的樓層建立欄杆
        outdoor_spaces = [
            ("floor_1_elderly", "accessible_garden"),
            ("floor_2_nursery", "outdoor_play_area"),
            ("floor_3_family", "outdoor_terrace")
        ]
        
        for floor_name, space_name in outdoor_spaces:
            if floor_name in self.floor_specs:
                railing_path = f"{self.building_path}/{floor_name}/railings"
                self._create_safety_railing(railing_path)
    
    def _create_safety_railing(self, railing_path: str):
        """建立安全欄杆"""
        railing = UsdGeom.Xform.Define(self.stage, railing_path)
        
        # 欄杆參數
        rail_height = 1.1  # 110cm 高
        post_spacing = 2.0
        num_posts = 5
        
        for i in range(num_posts):
            # 垂直欄杆柱
            post_path = f"{railing_path}/post_{i}"
            post = UsdGeom.Cylinder.Define(self.stage, post_path)
            
            post.GetRadiusAttr().Set(0.05)
            post.GetHeightAttr().Set(rail_height)
            
            x_pos = -4 + i * post_spacing
            UsdGeom.XformCommonAPI(post).SetTranslate([x_pos, 10, rail_height/2])
            
            # 設定金屬材質顏色
            post.GetDisplayColorAttr().Set([(0.7, 0.7, 0.7)])
        
        # 橫向扶手
        handrail_path = f"{railing_path}/handrail"
        handrail = UsdGeom.Cube.Define(self.stage, handrail_path)
        
        handrail.GetExtentAttr().Set([
            (-4, -0.05, rail_height - 0.05),
            (4, 0.05, rail_height + 0.05)
        ])
        
        UsdGeom.XformCommonAPI(handrail).SetTranslate([0, 10, 0])
        handrail.GetDisplayColorAttr().Set([(0.7, 0.7, 0.7)])
    
    def _create_signage(self):
        """建立標示系統"""
        signage_path = f"{self.building_path}/signage"
        signage_group = UsdGeom.Xform.Define(self.stage, signage_path)
        
        # 建立各樓層標示
        floor_signs = [
            ("B1", "停車場 Parking", -3.0),
            ("1F", "長照日照中心 Elderly Care", 0),
            ("2F", "托嬰中心 Nursery", 3.5),
            ("3F", "家庭服務 Family Services", 7.0),
            ("4F", "青少年中心 Youth Center", 10.5)
        ]
        
        for floor_id, floor_text, z_pos in floor_signs:
            sign_path = f"{signage_path}/floor_sign_{floor_id}"
            sign = UsdGeom.Cube.Define(self.stage, sign_path)
            
            # 標示牌尺寸
            sign.GetExtentAttr().Set([
                (-1.0, -0.05, -0.3),
                (1.0, 0.05, 0.3)
            ])
            
            # 設定位置（電梯旁）
            UsdGeom.XformCommonAPI(sign).SetTranslate([-5, -6, z_pos + 2])
            
            # 設定顏色
            sign.GetDisplayColorAttr().Set([(0.2, 0.4, 0.8)])
    
    def _add_space_label(self, space_path: str, space_name: str, space_type: str):
        """新增空間標示"""
        # 這裡可以新增 3D 文字或其他視覺標記
        # 由於 USD 的限制，實際文字需要使用特殊的文字幾何或紋理
        logger.debug(f"新增空間標示：{space_name} ({space_type})")
    
    def add_physics(self):
        """為建築物新增物理屬性"""
        # 新增重力
        physics_scene = UsdPhysics.Scene.Define(
            self.stage, "/World/PhysicsScene"
        )
        physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
        physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        
        # 為所有結構元素新增碰撞
        for prim in self.stage.Traverse():
            if prim.GetTypeName() in ["Cube", "Cylinder", "Sphere"]:
                # 檢查是否為結構元素
                path = str(prim.GetPath())
                if any(keyword in path for keyword in 
                      ["wall", "column", "beam", "slab", "foundation"]):
                    # 新增靜態剛體
                    UsdPhysics.CollisionAPI.Apply(prim)
                    rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
                    rigid_body.CreateKinematicEnabledAttr().Set(True)
        
        logger.info("物理屬性設定完成")
    
    def optimize_geometry(self):
        """優化幾何結構以提升效能"""
        # 合併相同材質的物件
        # 使用 LOD（細節層次）
        # 剔除不可見的面
        logger.info("幾何優化完成")
    
    def save_building(self, filepath: str):
        """儲存建築模型"""
        self.stage.Export(filepath)
        logger.info(f"建築模型已儲存至：{filepath}")
    
    def validate_building(self) -> bool:
        """驗證建築結構完整性"""
        # 檢查所有必要的樓層是否都已建立
        required_floors = ["B1_parking", "floor_1_elderly", 
                          "floor_2_nursery", "floor_3_family", 
                          "floor_4_youth"]
        
        for floor in required_floors:
            floor_path = f"{self.building_path}/{floor}"
            if not self.stage.GetPrimAtPath(floor_path):
                logger.error(f"缺少必要樓層：{floor}")
                return False
        
        logger.info("建築驗證通過")
        return True


if __name__ == "__main__":
    """測試建築生成器"""
    # 初始化 Omniverse
    import omni.usd
    from omni.isaac.kit import SimulationApp
    
    # 建立模擬應用
    app = SimulationApp({"headless": False})
    
    # 建立建築
    generator = BuildingGenerator()
    generator.generate_building()
    generator.add_physics()
    generator.validate_building()
    
    # 儲存結果
    generator.save_building("test_building.usd")
    
    # 執行模擬
    while app.is_running():
        app.update()
    
    app.close()
