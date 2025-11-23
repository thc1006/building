#!/usr/bin/env python3
"""
設施配置器 - 在建築物中配置家具和設備
Facility Placer for Easterlin Building
"""

import omni
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf, UsdGeom, Sdf
import yaml
import random
import numpy as np
from typing import Dict, List, Tuple, Optional
import logging

logger = logging.getLogger(__name__)


class FacilityPlacer:
    """設施配置器類別"""
    
    def __init__(self, config_path: str = "configs/building_config.yaml"):
        """
        初始化設施配置器
        
        Args:
            config_path: 配置檔案路徑
        """
        self.stage = omni.usd.get_context().get_stage()
        self.config = self._load_config(config_path)
        
        # 資產路徑
        self.assets_root = get_assets_root_path()
        self.custom_assets_path = "assets/"
        
        # 初始化資產庫
        self.asset_library = self._init_asset_library()
        
        # 配置記錄
        self.placed_facilities = {}
        
        logger.info(f"設施配置器初始化完成，載入 {len(self.asset_library)} 種設施類型")
    
    def _load_config(self, config_path: str) -> Dict:
        """載入配置檔案"""
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            logger.warning(f"找不到配置檔案 {config_path}，使用預設配置")
            return {}
    
    def _init_asset_library(self) -> Dict:
        """初始化資產庫"""
        library = {
            # === 長照設施 (1F) ===
            "elderly_care": {
                "wheelchair": {
                    "path": "/Props/Medical/Wheelchair.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": True
                },
                "walker": {
                    "path": "/Props/Medical/Walker.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": True
                },
                "hospital_bed": {
                    "path": "/Props/Medical/HospitalBed.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "recliner_chair": {
                    "path": "/Props/Furniture/ReclinerChair.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "therapy_mat": {
                    "path": "/Props/Medical/TherapyMat.usd",
                    "scale": [2.0, 2.0, 0.1],
                    "physics": False
                },
                "exercise_bike": {
                    "path": "/Props/Gym/ExerciseBike.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "parallel_bars": {
                    "path": "/Props/Medical/ParallelBars.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                }
            },
            
            # === 托嬰設施 (2F) ===
            "nursery": {
                "crib": {
                    "path": "/Props/Nursery/Crib.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "changing_table": {
                    "path": "/Props/Nursery/ChangingTable.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "play_mat": {
                    "path": "/Props/Nursery/PlayMat.usd",
                    "scale": [3.0, 3.0, 0.1],
                    "physics": False
                },
                "toy_box": {
                    "path": "/Props/Nursery/ToyBox.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": True
                },
                "baby_gate": {
                    "path": "/Props/Nursery/BabyGate.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "high_chair": {
                    "path": "/Props/Nursery/HighChair.usd",
                    "scale": [0.8, 0.8, 1.0],
                    "physics": False
                },
                "soft_blocks": {
                    "path": "/Props/Nursery/SoftBlocks.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": True
                }
            },
            
            # === 家庭服務設施 (3F) ===
            "family_services": {
                "conference_table": {
                    "path": "/Props/Office/ConferenceTable.usd",
                    "scale": [2.0, 1.0, 1.0],
                    "physics": False
                },
                "projector": {
                    "path": "/Props/Office/Projector.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "kitchen_stove": {
                    "path": "/Props/Kitchen/CommercialStove.usd",
                    "scale": [1.2, 1.2, 1.0],
                    "physics": False
                },
                "prep_station": {
                    "path": "/Props/Kitchen/PrepStation.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "counseling_chair": {
                    "path": "/Props/Office/ComfortableChair.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                }
            },
            
            # === 青少年設施 (4F) ===
            "youth_center": {
                "basketball_hoop": {
                    "path": "/Props/Sports/BasketballHoop.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "study_desk": {
                    "path": "/Props/Office/StudyDesk.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "computer_station": {
                    "path": "/Props/Office/ComputerDesk.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "3d_printer": {
                    "path": "/Props/Tech/3DPrinter.usd",
                    "scale": [0.8, 0.8, 0.8],
                    "physics": False
                },
                "dance_mirror": {
                    "path": "/Props/Gym/DanceMirror.usd",
                    "scale": [3.0, 0.1, 2.5],
                    "physics": False
                },
                "gaming_console": {
                    "path": "/Props/Entertainment/GamingConsole.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": True
                },
                "pool_table": {
                    "path": "/Props/Entertainment/PoolTable.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                }
            },
            
            # === 通用設施 ===
            "common": {
                "chair": {
                    "path": "/Props/Furniture/Chair.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": True
                },
                "table": {
                    "path": "/Props/Furniture/Table.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "sofa": {
                    "path": "/Props/Furniture/Sofa.usd",
                    "scale": [1.5, 1.0, 1.0],
                    "physics": False
                },
                "cabinet": {
                    "path": "/Props/Furniture/Cabinet.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "plant_pot": {
                    "path": "/Props/Decoration/PlantPot.usd",
                    "scale": [0.8, 0.8, 0.8],
                    "physics": False
                },
                "water_dispenser": {
                    "path": "/Props/Appliances/WaterDispenser.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "trash_bin": {
                    "path": "/Props/Furniture/TrashBin.usd",
                    "scale": [0.8, 0.8, 0.8],
                    "physics": True
                }
            },
            
            # === 停車場設施 (B1) ===
            "parking": {
                "parking_sign": {
                    "path": "/Props/Signs/ParkingSign.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                },
                "safety_cone": {
                    "path": "/Props/Safety/TrafficCone.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": True
                },
                "fire_extinguisher": {
                    "path": "/Props/Safety/FireExtinguisher.usd",
                    "scale": [1.0, 1.0, 1.0],
                    "physics": False
                }
            }
        }
        
        return library
    
    def place_all_facilities(self):
        """配置所有設施"""
        logger.info("開始配置所有設施...")
        
        floors = self.config.get("floors", {})
        
        for floor_name, floor_config in floors.items():
            self.place_floor_facilities(floor_name, floor_config)
        
        logger.info(f"設施配置完成，共配置 {len(self.placed_facilities)} 個設施")
    
    def place_floor_facilities(self, floor_name: str, floor_config: Dict):
        """
        為特定樓層配置設施
        
        Args:
            floor_name: 樓層名稱
            floor_config: 樓層配置
        """
        logger.info(f"配置樓層設施：{floor_name}")
        
        spaces = floor_config.get("spaces", [])
        
        for space in spaces:
            self.place_space_facilities(floor_name, space)
    
    def place_space_facilities(self, floor_name: str, space_config: Dict):
        """
        為特定空間配置設施
        
        Args:
            floor_name: 樓層名稱
            space_config: 空間配置
        """
        space_name = space_config.get("name", "unnamed")
        space_type = space_config.get("type", "generic")
        position = space_config.get("position", [0, 0, 0])
        dimensions = space_config.get("dimensions", [10, 10, 3.5])
        
        logger.debug(f"配置空間 {space_name} (類型：{space_type})")
        
        # 根據空間類型選擇設施
        facilities = self._get_facilities_for_space(space_type, space_config)
        
        # 配置設施
        for facility_type, count in facilities.items():
            for i in range(count):
                self._place_single_facility(
                    floor_name,
                    space_name,
                    facility_type,
                    position,
                    dimensions,
                    i
                )
    
    def _get_facilities_for_space(self, space_type: str, 
                                 space_config: Dict) -> Dict[str, int]:
        """
        根據空間類型決定需要的設施
        
        Args:
            space_type: 空間類型
            space_config: 空間配置
            
        Returns:
            設施類型和數量的字典
        """
        facilities = {}
        
        if space_type == "care_unit":
            # 長照單元
            facilities = {
                "wheelchair": 5,
                "walker": 3,
                "recliner_chair": 10,
                "table": 5,
                "chair": 20
            }
            
        elif space_type == "nursery":
            # 托嬰室
            age_group = space_config.get("age_group", "0-2")
            if "0-1" in age_group:
                facilities = {
                    "crib": 15,
                    "changing_table": 2,
                    "play_mat": 3,
                    "soft_blocks": 5
                }
            else:
                facilities = {
                    "play_mat": 5,
                    "toy_box": 3,
                    "high_chair": 10
                }
                
        elif space_type == "sports":
            # 運動場地
            facilities = {
                "basketball_hoop": 2
            }
            
        elif space_type == "educational":
            # 教育空間
            facilities = {
                "study_desk": 30,
                "chair": 30,
                "computer_station": 10
            }
            
        elif space_type == "creative":
            # 創作空間
            facilities = {
                "3d_printer": 2,
                "table": 4,
                "chair": 12
            }
            
        elif space_type == "dining":
            # 餐廳
            capacity = space_config.get("capacity", 60)
            facilities = {
                "table": capacity // 6,  # 每桌6人
                "chair": capacity
            }
            
        elif space_type == "kitchen":
            # 廚房
            facilities = {
                "kitchen_stove": 2,
                "prep_station": 4
            }
            
        elif space_type == "office":
            # 辦公室
            facilities = {
                "counseling_chair": 6,
                "table": 3,
                "cabinet": 2
            }
            
        elif space_type == "parking":
            # 停車場
            facilities = {
                "parking_sign": 10,
                "safety_cone": 5,
                "fire_extinguisher": 4
            }
            
        else:
            # 通用空間
            facilities = {
                "chair": 10,
                "table": 3,
                "plant_pot": 2
            }
        
        return facilities
    
    def _place_single_facility(self, floor_name: str, space_name: str,
                              facility_type: str, space_position: List[float],
                              space_dimensions: List[float], index: int):
        """
        放置單一設施
        
        Args:
            floor_name: 樓層名稱
            space_name: 空間名稱
            facility_type: 設施類型
            space_position: 空間位置
            space_dimensions: 空間尺寸
            index: 設施索引
        """
        # 查找資產
        asset_info = None
        for category, items in self.asset_library.items():
            if facility_type in items:
                asset_info = items[facility_type]
                break
        
        if not asset_info:
            # 如果找不到特定資產，使用通用資產
            if facility_type in self.asset_library["common"]:
                asset_info = self.asset_library["common"][facility_type]
            else:
                logger.warning(f"找不到設施類型：{facility_type}")
                return
        
        # 計算設施位置
        position = self._calculate_facility_position(
            space_position,
            space_dimensions,
            facility_type,
            index
        )
        
        # 建立設施路徑
        prim_path = f"/World/Building/{floor_name}/{space_name}/{facility_type}_{index}"
        
        # 檢查路徑是否已存在
        if self.stage.GetPrimAtPath(prim_path):
            logger.debug(f"設施已存在：{prim_path}")
            return
        
        # 建立設施
        if asset_info["path"].startswith("/Props/"):
            # 使用預設資產（建立簡單幾何代替）
            self._create_placeholder_facility(
                prim_path,
                facility_type,
                position,
                asset_info.get("scale", [1, 1, 1])
            )
        else:
            # 載入 USD 檔案
            self._load_usd_asset(
                prim_path,
                asset_info["path"],
                position,
                asset_info.get("scale", [1, 1, 1])
            )
        
        # 新增物理屬性
        if asset_info.get("physics", False):
            self._add_physics_to_facility(prim_path)
        
        # 記錄已配置的設施
        if floor_name not in self.placed_facilities:
            self.placed_facilities[floor_name] = {}
        if space_name not in self.placed_facilities[floor_name]:
            self.placed_facilities[floor_name][space_name] = []
        
        self.placed_facilities[floor_name][space_name].append({
            "type": facility_type,
            "path": prim_path,
            "position": position
        })
    
    def _calculate_facility_position(self, space_position: List[float],
                                    space_dimensions: List[float],
                                    facility_type: str, index: int) -> List[float]:
        """
        計算設施位置
        
        Args:
            space_position: 空間中心位置
            space_dimensions: 空間尺寸
            facility_type: 設施類型
            index: 設施索引
            
        Returns:
            設施的3D位置
        """
        # 使用網格佈局
        grid_size = self._get_grid_size(facility_type)
        
        # 計算網格位置
        row = index // grid_size[0]
        col = index % grid_size[0]
        
        # 計算相對位置（在空間內）
        x_spacing = space_dimensions[0] / (grid_size[0] + 1)
        y_spacing = space_dimensions[1] / (grid_size[1] + 1)
        
        x_offset = -space_dimensions[0]/2 + (col + 1) * x_spacing
        y_offset = -space_dimensions[1]/2 + (row + 1) * y_spacing
        
        # 加入一些隨機偏移以避免過於規則
        if facility_type not in ["study_desk", "computer_station", "crib"]:
            x_offset += random.uniform(-0.3, 0.3)
            y_offset += random.uniform(-0.3, 0.3)
        
        # 計算最終位置
        position = [
            space_position[0] + x_offset,
            space_position[1] + y_offset,
            space_position[2]
        ]
        
        return position
    
    def _get_grid_size(self, facility_type: str) -> Tuple[int, int]:
        """
        根據設施類型決定網格大小
        
        Args:
            facility_type: 設施類型
            
        Returns:
            (列數, 行數)
        """
        grid_sizes = {
            "study_desk": (6, 5),
            "computer_station": (5, 4),
            "crib": (5, 3),
            "table": (3, 3),
            "chair": (6, 5),
            "wheelchair": (3, 2),
            "recliner_chair": (4, 3)
        }
        
        return grid_sizes.get(facility_type, (4, 4))
    
    def _create_placeholder_facility(self, prim_path: str, facility_type: str,
                                    position: List[float], scale: List[float]):
        """
        建立佔位設施（當實際模型不可用時）
        
        Args:
            prim_path: Prim 路徑
            facility_type: 設施類型
            position: 位置
            scale: 縮放
        """
        # 根據設施類型選擇合適的幾何形狀
        if "chair" in facility_type:
            # 椅子 - 使用方塊
            prim = UsdGeom.Cube.Define(self.stage, prim_path)
            prim.GetExtentAttr().Set([
                (-0.25, -0.25, 0),
                (0.25, 0.25, 0.8)
            ])
            prim.GetDisplayColorAttr().Set([(0.6, 0.4, 0.2)])
            
        elif "table" in facility_type or "desk" in facility_type:
            # 桌子 - 使用扁平方塊
            prim = UsdGeom.Cube.Define(self.stage, prim_path)
            prim.GetExtentAttr().Set([
                (-0.6, -0.4, 0),
                (0.6, 0.4, 0.75)
            ])
            prim.GetDisplayColorAttr().Set([(0.7, 0.5, 0.3)])
            
        elif "bed" in facility_type or "crib" in facility_type:
            # 床 - 使用長方塊
            prim = UsdGeom.Cube.Define(self.stage, prim_path)
            prim.GetExtentAttr().Set([
                (-0.5, -1.0, 0),
                (0.5, 1.0, 0.5)
            ])
            prim.GetDisplayColorAttr().Set([(0.9, 0.9, 0.9)])
            
        elif "hoop" in facility_type:
            # 籃球架 - 使用圓柱和圓環組合
            group = UsdGeom.Xform.Define(self.stage, prim_path)
            
            # 支柱
            pole_path = f"{prim_path}/pole"
            pole = UsdGeom.Cylinder.Define(self.stage, pole_path)
            pole.GetRadiusAttr().Set(0.1)
            pole.GetHeightAttr().Set(3.0)
            UsdGeom.XformCommonAPI(pole).SetTranslate([0, 0, 1.5])
            
            # 籃板
            board_path = f"{prim_path}/board"
            board = UsdGeom.Cube.Define(self.stage, board_path)
            board.GetExtentAttr().Set([
                (-0.9, -0.02, 2.5),
                (0.9, 0.02, 3.5)
            ])
            
            # 籃框（用圓環近似）
            rim_path = f"{prim_path}/rim"
            rim = UsdGeom.Cylinder.Define(self.stage, rim_path)
            rim.GetRadiusAttr().Set(0.225)  # 標準籃框半徑
            rim.GetHeightAttr().Set(0.02)
            UsdGeom.XformCommonAPI(rim).SetTranslate([0, 0.3, 3.05])
            rim.AddRotateXOp().Set(90)
            
            prim = group
            
        else:
            # 預設 - 使用方塊
            prim = UsdGeom.Cube.Define(self.stage, prim_path)
            prim.GetExtentAttr().Set([
                (-0.3, -0.3, 0),
                (0.3, 0.3, 0.6)
            ])
            prim.GetDisplayColorAttr().Set([(0.5, 0.5, 0.5)])
        
        # 設定位置
        if hasattr(prim, 'AddTranslateOp'):
            prim.AddTranslateOp().Set(Gf.Vec3f(*position))
        else:
            UsdGeom.XformCommonAPI(prim).SetTranslate(position)
        
        # 設定縮放
        if hasattr(prim, 'AddScaleOp'):
            prim.AddScaleOp().Set(Gf.Vec3f(*scale))
    
    def _load_usd_asset(self, prim_path: str, asset_path: str,
                       position: List[float], scale: List[float]):
        """
        載入 USD 資產檔案
        
        Args:
            prim_path: Prim 路徑
            asset_path: 資產檔案路徑
            position: 位置
            scale: 縮放
        """
        try:
            # 檢查檔案是否存在
            full_path = self.custom_assets_path + asset_path
            
            # 建立參考
            prim = self.stage.DefinePrim(prim_path)
            prim.GetReferences().AddReference(full_path)
            
            # 設定變換
            xform = UsdGeom.XformCommonAPI(prim)
            xform.SetTranslate(position)
            xform.SetScale(scale)
            
        except Exception as e:
            logger.warning(f"無法載入資產 {asset_path}：{e}")
            # 改用佔位設施
            self._create_placeholder_facility(
                prim_path,
                prim_path.split("/")[-1].split("_")[0],
                position,
                scale
            )
    
    def _add_physics_to_facility(self, prim_path: str):
        """
        為設施新增物理屬性
        
        Args:
            prim_path: Prim 路徑
        """
        from pxr import UsdPhysics
        
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim:
            return
        
        # 新增剛體
        UsdPhysics.RigidBodyAPI.Apply(prim)
        
        # 新增碰撞
        UsdPhysics.CollisionAPI.Apply(prim)
        
        # 設定質量（根據設施類型）
        facility_type = prim_path.split("/")[-1].split("_")[0]
        mass = self._get_facility_mass(facility_type)
        
        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        rigid_body.CreateMassAttr().Set(mass)
    
    def _get_facility_mass(self, facility_type: str) -> float:
        """
        根據設施類型返回質量
        
        Args:
            facility_type: 設施類型
            
        Returns:
            質量（公斤）
        """
        masses = {
            "chair": 5.0,
            "table": 20.0,
            "desk": 30.0,
            "wheelchair": 15.0,
            "walker": 3.0,
            "crib": 25.0,
            "sofa": 50.0,
            "cabinet": 40.0
        }
        
        return masses.get(facility_type, 10.0)
    
    def rearrange_facilities(self, floor_name: str, space_name: str,
                           layout_type: str = "grid"):
        """
        重新排列設施
        
        Args:
            floor_name: 樓層名稱
            space_name: 空間名稱
            layout_type: 佈局類型 (grid, circle, random)
        """
        if floor_name not in self.placed_facilities:
            logger.warning(f"找不到樓層：{floor_name}")
            return
        
        if space_name not in self.placed_facilities[floor_name]:
            logger.warning(f"找不到空間：{space_name}")
            return
        
        facilities = self.placed_facilities[floor_name][space_name]
        
        if layout_type == "grid":
            self._arrange_grid(facilities)
        elif layout_type == "circle":
            self._arrange_circle(facilities)
        elif layout_type == "random":
            self._arrange_random(facilities)
    
    def _arrange_grid(self, facilities: List[Dict]):
        """網格排列"""
        grid_size = int(np.ceil(np.sqrt(len(facilities))))
        
        for i, facility in enumerate(facilities):
            row = i // grid_size
            col = i % grid_size
            
            new_position = [
                col * 2.0 - grid_size,
                row * 2.0 - grid_size,
                facility["position"][2]
            ]
            
            prim = self.stage.GetPrimAtPath(facility["path"])
            if prim:
                UsdGeom.XformCommonAPI(prim).SetTranslate(new_position)
    
    def _arrange_circle(self, facilities: List[Dict]):
        """圓形排列"""
        radius = 5.0
        angle_step = 2 * np.pi / len(facilities)
        
        for i, facility in enumerate(facilities):
            angle = i * angle_step
            
            new_position = [
                radius * np.cos(angle),
                radius * np.sin(angle),
                facility["position"][2]
            ]
            
            prim = self.stage.GetPrimAtPath(facility["path"])
            if prim:
                UsdGeom.XformCommonAPI(prim).SetTranslate(new_position)
    
    def _arrange_random(self, facilities: List[Dict]):
        """隨機排列"""
        for facility in facilities:
            new_position = [
                random.uniform(-10, 10),
                random.uniform(-10, 10),
                facility["position"][2]
            ]
            
            prim = self.stage.GetPrimAtPath(facility["path"])
            if prim:
                UsdGeom.XformCommonAPI(prim).SetTranslate(new_position)
    
    def get_facility_statistics(self) -> Dict:
        """取得設施統計資料"""
        stats = {
            "total_facilities": 0,
            "by_floor": {},
            "by_type": {}
        }
        
        for floor_name, spaces in self.placed_facilities.items():
            floor_count = 0
            for space_name, facilities in spaces.items():
                floor_count += len(facilities)
                
                for facility in facilities:
                    facility_type = facility["type"]
                    if facility_type not in stats["by_type"]:
                        stats["by_type"][facility_type] = 0
                    stats["by_type"][facility_type] += 1
            
            stats["by_floor"][floor_name] = floor_count
            stats["total_facilities"] += floor_count
        
        return stats
    
    def validate_placement(self) -> List[str]:
        """驗證設施配置是否合理"""
        issues = []
        
        # 檢查必要設施
        required_facilities = {
            "floor_1_elderly": ["wheelchair", "walker"],
            "floor_2_nursery": ["crib", "changing_table"],
            "floor_4_youth": ["study_desk"]
        }
        
        for floor_name, required in required_facilities.items():
            if floor_name in self.placed_facilities:
                placed_types = []
                for space in self.placed_facilities[floor_name].values():
                    placed_types.extend([f["type"] for f in space])
                
                for required_type in required:
                    if required_type not in placed_types:
                        issues.append(f"{floor_name} 缺少必要設施：{required_type}")
        
        # 檢查設施間距
        # （簡化版本，實際應該檢查碰撞）
        
        return issues


if __name__ == "__main__":
    """測試設施配置器"""
    # 初始化 Omniverse
    import omni.usd
    from omni.isaac.kit import SimulationApp
    
    # 建立模擬應用
    app = SimulationApp({"headless": False})
    
    # 建立設施配置器
    placer = FacilityPlacer()
    
    # 配置所有設施
    placer.place_all_facilities()
    
    # 取得統計資料
    stats = placer.get_facility_statistics()
    print(f"設施統計：{stats}")
    
    # 驗證配置
    issues = placer.validate_placement()
    if issues:
        print(f"配置問題：{issues}")
    else:
        print("設施配置驗證通過")
    
    # 執行模擬
    while app.is_running():
        app.update()
    
    app.close()
