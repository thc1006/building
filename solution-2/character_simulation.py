#!/usr/bin/env python3
"""
人物模擬系統 - 模擬建築物中的各類人員
Character Simulation System for Easterlin Building
"""

import omni
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import quat_to_euler_angles, euler_to_quat
from pxr import Gf, UsdGeom, UsdSkel, Sdf
import numpy as np
import random
from datetime import datetime, timedelta
from typing import Dict, List, Tuple, Optional
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class CharacterType(Enum):
    """人物類型枚舉"""
    ELDERLY = "elderly"           # 長者
    TODDLER = "toddler"          # 幼兒
    YOUTH = "youth"               # 青少年
    CAREGIVER = "caregiver"       # 照護人員
    PARENT = "parent"             # 家長
    VOLUNTEER = "volunteer"       # 志工
    STAFF = "staff"               # 行政人員


class ActivityType(Enum):
    """活動類型枚舉"""
    # 通用活動
    WALKING = "walking"
    SITTING = "sitting"
    EATING = "eating"
    RESTING = "resting"
    SOCIALIZING = "socializing"
    
    # 長者活動
    EXERCISE = "exercise"
    REHABILITATION = "rehabilitation"
    GARDENING = "gardening"
    WANDERING = "wandering"
    
    # 幼兒活動
    PLAYING = "playing"
    CRAWLING = "crawling"
    SLEEPING = "sleeping"
    LEARNING = "learning"
    
    # 青少年活動
    STUDYING = "studying"
    SPORTS = "sports"
    GAMING = "gaming"
    CREATING = "creating"
    
    # 照護活動
    CARING = "caring"
    FEEDING = "feeding"
    MONITORING = "monitoring"
    EMERGENCY = "emergency"


class CharacterSimulation:
    """人物模擬主類別"""
    
    def __init__(self, config_path: str = "configs/building_config.yaml"):
        """
        初始化人物模擬系統
        
        Args:
            config_path: 配置檔案路徑
        """
        self.stage = omni.usd.get_context().get_stage()
        self.characters = {}
        self.behavior_patterns = {}
        self.current_time = datetime.now().replace(hour=9, minute=0)
        
        # 載入配置
        self.config = self._load_config(config_path)
        
        # 初始化行為模式
        self._init_behavior_patterns()
        
        logger.info("人物模擬系統初始化完成")
    
    def _load_config(self, config_path: str) -> Dict:
        """載入配置檔案"""
        import yaml
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            logger.warning(f"找不到配置檔案 {config_path}，使用預設配置")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict:
        """取得預設配置"""
        return {
            "characters": {
                "elderly": {"count": 50, "floor": "1F"},
                "toddlers": {"count": 40, "floor": "2F"},
                "youth": {"count": 30, "floor": "4F"},
                "caregivers": {"count": 15, "distribution": {}}
            }
        }
    
    def _init_behavior_patterns(self):
        """初始化行為模式"""
        self.behavior_patterns = {
            CharacterType.ELDERLY: self._get_elderly_behaviors(),
            CharacterType.TODDLER: self._get_toddler_behaviors(),
            CharacterType.YOUTH: self._get_youth_behaviors(),
            CharacterType.CAREGIVER: self._get_caregiver_behaviors()
        }
    
    def _get_elderly_behaviors(self) -> Dict:
        """長者行為模式"""
        return {
            "daily_schedule": [
                {"time": "09:00", "activity": ActivityType.EXERCISE, "location": "activity_room", "duration": 60},
                {"time": "10:00", "activity": ActivityType.SOCIALIZING, "location": "common_area", "duration": 30},
                {"time": "10:30", "activity": ActivityType.RESTING, "location": "rest_area", "duration": 30},
                {"time": "11:00", "activity": ActivityType.REHABILITATION, "location": "rehab_room", "duration": 45},
                {"time": "11:45", "activity": ActivityType.EATING, "location": "dining", "duration": 45},
                {"time": "12:30", "activity": ActivityType.RESTING, "location": "rest_area", "duration": 90},
                {"time": "14:00", "activity": ActivityType.SOCIALIZING, "location": "common_area", "duration": 60},
                {"time": "15:00", "activity": ActivityType.GARDENING, "location": "garden", "duration": 60},
                {"time": "16:00", "activity": ActivityType.SITTING, "location": "lobby", "duration": 60}
            ],
            "mobility": {
                "independent": 0.6,
                "walker": 0.25,
                "wheelchair": 0.15
            },
            "dementia_behaviors": {
                "wandering_probability": 0.1,
                "confusion_probability": 0.05,
                "agitation_probability": 0.03
            }
        }
    
    def _get_toddler_behaviors(self) -> Dict:
        """幼兒行為模式"""
        return {
            "daily_schedule": [
                {"time": "07:30", "activity": ActivityType.PLAYING, "location": "play_area", "duration": 90},
                {"time": "09:00", "activity": ActivityType.EATING, "location": "dining", "duration": 30},
                {"time": "09:30", "activity": ActivityType.LEARNING, "location": "classroom", "duration": 60},
                {"time": "10:30", "activity": ActivityType.PLAYING, "location": "outdoor", "duration": 60},
                {"time": "11:30", "activity": ActivityType.EATING, "location": "dining", "duration": 45},
                {"time": "12:15", "activity": ActivityType.SLEEPING, "location": "nap_room", "duration": 120},
                {"time": "14:15", "activity": ActivityType.PLAYING, "location": "play_area", "duration": 75},
                {"time": "15:30", "activity": ActivityType.EATING, "location": "dining", "duration": 30},
                {"time": "16:00", "activity": ActivityType.PLAYING, "location": "play_area", "duration": 90}
            ],
            "age_groups": {
                "0-1": {"crawling": 0.7, "walking": 0.3},
                "1-2": {"crawling": 0.1, "walking": 0.9}
            },
            "emotional_states": {
                "happy": 0.6,
                "neutral": 0.3,
                "fussy": 0.08,
                "crying": 0.02
            }
        }
    
    def _get_youth_behaviors(self) -> Dict:
        """青少年行為模式"""
        return {
            "daily_schedule": [
                {"time": "17:00", "activity": ActivityType.SPORTS, "location": "basketball_court", "duration": 90},
                {"time": "18:30", "activity": ActivityType.EATING, "location": "dining", "duration": 30},
                {"time": "19:00", "activity": ActivityType.STUDYING, "location": "study_room", "duration": 90},
                {"time": "20:30", "activity": ActivityType.SOCIALIZING, "location": "social_area", "duration": 30}
            ],
            "interests": {
                "sports": 0.4,
                "study": 0.35,
                "creative": 0.15,
                "gaming": 0.1
            },
            "social_groups": {
                "group_size": [2, 5],  # 最小和最大群組大小
                "interaction_frequency": 0.7
            }
        }
    
    def _get_caregiver_behaviors(self) -> Dict:
        """照護人員行為模式"""
        return {
            "work_schedule": {
                "morning_shift": {"start": "07:00", "end": "15:00"},
                "afternoon_shift": {"start": "13:00", "end": "21:00"},
                "admin": {"start": "09:00", "end": "17:00"}
            },
            "tasks": {
                "routine_care": 0.4,
                "medical_assistance": 0.2,
                "activity_facilitation": 0.2,
                "documentation": 0.1,
                "emergency_response": 0.1
            },
            "patrol_routes": [
                ["nurse_station", "dementia_area", "activity_room", "nurse_station"],
                ["nurse_station", "dining", "rest_area", "nurse_station"],
                ["nurse_station", "garden", "rehabilitation", "nurse_station"]
            ]
        }
    
    def spawn_all_characters(self):
        """生成所有人物"""
        logger.info("開始生成所有人物...")
        
        characters_config = self.config.get("characters", {})
        
        # 生成長者
        self._spawn_character_group(
            CharacterType.ELDERLY,
            characters_config.get("elderly", {})
        )
        
        # 生成幼兒
        self._spawn_character_group(
            CharacterType.TODDLER,
            characters_config.get("toddlers", {})
        )
        
        # 生成青少年
        self._spawn_character_group(
            CharacterType.YOUTH,
            characters_config.get("youth", {})
        )
        
        # 生成照護人員
        self._spawn_character_group(
            CharacterType.CAREGIVER,
            characters_config.get("caregivers", {})
        )
        
        logger.info(f"人物生成完成，共生成 {len(self.characters)} 個角色")
    
    def _spawn_character_group(self, char_type: CharacterType, config: Dict):
        """
        生成特定類型的人物群組
        
        Args:
            char_type: 人物類型
            config: 配置資訊
        """
        # 決定生成數量
        if "count" in config:
            if isinstance(config["count"], str) and "-" in config["count"]:
                min_count, max_count = map(int, config["count"].split("-"))
                count = random.randint(min_count, max_count)
            else:
                count = int(config["count"])
        else:
            count = 10  # 預設數量
        
        logger.info(f"生成 {count} 個 {char_type.value} 類型角色")
        
        for i in range(count):
            char_id = f"{char_type.value}_{i:03d}"
            character = self._create_character(char_id, char_type, config)
            self.characters[char_id] = character
    
    def _create_character(self, char_id: str, char_type: CharacterType, 
                         config: Dict) -> Dict:
        """
        建立單一人物
        
        Args:
            char_id: 人物ID
            char_type: 人物類型
            config: 配置資訊
            
        Returns:
            人物資料字典
        """
        # 建立人物路徑
        char_path = f"/World/Characters/{char_type.value}/{char_id}"
        
        # 建立人物 Prim
        char_prim = self._create_character_prim(char_path, char_type)
        
        # 決定初始位置
        initial_position = self._get_spawn_position(char_type, config)
        
        # 建立人物資料
        character = {
            "id": char_id,
            "type": char_type,
            "path": char_path,
            "prim": char_prim,
            "position": initial_position,
            "velocity": [0, 0, 0],
            "orientation": 0,  # 朝向角度
            "current_activity": None,
            "target_position": None,
            "schedule": self._generate_personal_schedule(char_type),
            "attributes": self._generate_character_attributes(char_type),
            "state": self._init_character_state(char_type),
            "interaction_history": [],
            "health_status": "healthy"
        }
        
        # 設定初始位置
        self._set_character_position(char_path, initial_position)
        
        # 如果是長者，可能需要輔助設備
        if char_type == CharacterType.ELDERLY:
            self._add_mobility_aid(character)
        
        return character
    
    def _create_character_prim(self, char_path: str, 
                              char_type: CharacterType) -> any:
        """
        建立人物 Prim（簡化版使用圓柱或膠囊體表示）
        
        Args:
            char_path: 人物路徑
            char_type: 人物類型
            
        Returns:
            Prim 物件
        """
        # 建立群組
        char_xform = UsdGeom.Xform.Define(self.stage, char_path)
        
        # 建立身體（使用膠囊體或圓柱體）
        body_path = f"{char_path}/body"
        
        if char_type == CharacterType.TODDLER:
            # 幼兒 - 較小的身形
            body = UsdGeom.Cylinder.Define(self.stage, body_path)
            body.GetRadiusAttr().Set(0.15)
            body.GetHeightAttr().Set(0.8)
            color = [(1.0, 0.8, 0.6)]  # 膚色
            
        elif char_type == CharacterType.ELDERLY:
            # 長者 - 標準成人身形
            body = UsdGeom.Cylinder.Define(self.stage, body_path)
            body.GetRadiusAttr().Set(0.25)
            body.GetHeightAttr().Set(1.6)
            color = [(0.7, 0.7, 0.8)]  # 灰色衣服
            
        elif char_type == CharacterType.YOUTH:
            # 青少年 - 較高的身形
            body = UsdGeom.Cylinder.Define(self.stage, body_path)
            body.GetRadiusAttr().Set(0.22)
            body.GetHeightAttr().Set(1.7)
            color = [(0.3, 0.5, 0.8)]  # 藍色衣服
            
        else:  # 照護人員
            body = UsdGeom.Cylinder.Define(self.stage, body_path)
            body.GetRadiusAttr().Set(0.25)
            body.GetHeightAttr().Set(1.65)
            color = [(0.9, 0.9, 1.0)]  # 白色制服
        
        body.GetDisplayColorAttr().Set(color)
        
        # 建立頭部
        head_path = f"{char_path}/head"
        head = UsdGeom.Sphere.Define(self.stage, head_path)
        
        if char_type == CharacterType.TODDLER:
            head.GetRadiusAttr().Set(0.12)
            head_height = 0.9
        else:
            head.GetRadiusAttr().Set(0.15)
            head_height = body.GetHeightAttr().Get() + 0.15
        
        UsdGeom.XformCommonAPI(head).SetTranslate([0, 0, head_height])
        head.GetDisplayColorAttr().Set([(1.0, 0.8, 0.6)])  # 膚色
        
        # 新增物理屬性
        self._add_character_physics(char_path)
        
        return char_xform
    
    def _add_character_physics(self, char_path: str):
        """為人物新增物理屬性"""
        from pxr import UsdPhysics
        
        char_prim = self.stage.GetPrimAtPath(char_path)
        
        # 新增剛體
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(char_prim)
        rigid_body.CreateMassAttr().Set(70.0)  # 70kg 平均體重
        
        # 新增碰撞
        collision = UsdPhysics.CollisionAPI.Apply(char_prim)
        
        # 防止傾倒（鎖定旋轉）
        rigid_body.CreateAngularVelocityAttr().Set([0, 0, 0])
    
    def _get_spawn_position(self, char_type: CharacterType, 
                          config: Dict) -> List[float]:
        """
        取得生成位置
        
        Args:
            char_type: 人物類型
            config: 配置資訊
            
        Returns:
            3D 位置座標
        """
        # 根據樓層決定 Z 座標
        floor_z = {
            "B1": -3.0,
            "1F": 0.0,
            "2F": 3.5,
            "3F": 7.0,
            "4F": 10.5
        }
        
        floor = config.get("floor", "1F")
        z = floor_z.get(floor, 0.0)
        
        # 在樓層範圍內隨機生成 X, Y
        x = random.uniform(-15, 15)
        y = random.uniform(-10, 10)
        
        # 確保不在牆內
        if abs(x) > 18:
            x = 18 * (1 if x > 0 else -1)
        if abs(y) > 11:
            y = 11 * (1 if y > 0 else -1)
        
        return [x, y, z]
    
    def _generate_personal_schedule(self, char_type: CharacterType) -> List[Dict]:
        """
        生成個人化時程表
        
        Args:
            char_type: 人物類型
            
        Returns:
            時程表列表
        """
        base_schedule = self.behavior_patterns[char_type]["daily_schedule"]
        
        # 加入一些隨機變化
        personal_schedule = []
        for activity in base_schedule:
            # 時間偏移 ±15分鐘
            time_offset = random.randint(-15, 15)
            original_time = datetime.strptime(activity["time"], "%H:%M")
            new_time = original_time + timedelta(minutes=time_offset)
            
            personal_activity = activity.copy()
            personal_activity["time"] = new_time.strftime("%H:%M")
            
            # 持續時間變化 ±10分鐘
            if "duration" in personal_activity:
                personal_activity["duration"] += random.randint(-10, 10)
            
            personal_schedule.append(personal_activity)
        
        return personal_schedule
    
    def _generate_character_attributes(self, char_type: CharacterType) -> Dict:
        """
        生成人物屬性
        
        Args:
            char_type: 人物類型
            
        Returns:
            屬性字典
        """
        attributes = {}
        
        if char_type == CharacterType.ELDERLY:
            attributes["age"] = random.randint(65, 95)
            attributes["mobility"] = random.choices(
                ["independent", "walker", "wheelchair"],
                weights=[0.6, 0.25, 0.15]
            )[0]
            attributes["cognitive_status"] = random.choices(
                ["normal", "mild_impairment", "dementia"],
                weights=[0.7, 0.2, 0.1]
            )[0]
            
        elif char_type == CharacterType.TODDLER:
            attributes["age_months"] = random.randint(0, 24)
            attributes["walking_ability"] = attributes["age_months"] > 12
            attributes["temperament"] = random.choice(["easy", "active", "shy"])
            
        elif char_type == CharacterType.YOUTH:
            attributes["age"] = random.randint(12, 18)
            attributes["interests"] = random.sample(
                ["sports", "study", "art", "music", "gaming"],
                k=random.randint(1, 3)
            )
            attributes["academic_level"] = random.choice(["high", "medium", "struggling"])
            
        elif char_type == CharacterType.CAREGIVER:
            attributes["experience_years"] = random.randint(1, 20)
            attributes["specialty"] = random.choice(
                ["elderly_care", "child_care", "rehabilitation", "nursing"]
            )
            attributes["shift"] = random.choice(["morning", "afternoon", "admin"])
        
        # 共通屬性
        attributes["personality"] = self._generate_personality()
        attributes["energy_level"] = random.uniform(0.5, 1.0)
        attributes["social_preference"] = random.choice(["social", "neutral", "solitary"])
        
        return attributes
    
    def _generate_personality(self) -> Dict:
        """生成人格特質（簡化的五大人格）"""
        return {
            "openness": random.uniform(0, 1),
            "conscientiousness": random.uniform(0, 1),
            "extraversion": random.uniform(0, 1),
            "agreeableness": random.uniform(0, 1),
            "neuroticism": random.uniform(0, 1)
        }
    
    def _init_character_state(self, char_type: CharacterType) -> Dict:
        """
        初始化人物狀態
        
        Args:
            char_type: 人物類型
            
        Returns:
            狀態字典
        """
        return {
            "energy": 1.0,
            "hunger": 0.0,
            "comfort": 1.0,
            "social_need": 0.5,
            "stress": 0.0,
            "happiness": 0.7,
            "health": 1.0,
            "alertness": 1.0 if char_type != CharacterType.TODDLER else 0.8
        }
    
    def _add_mobility_aid(self, character: Dict):
        """
        為長者新增行動輔具
        
        Args:
            character: 人物資料
        """
        mobility = character["attributes"].get("mobility", "independent")
        
        if mobility == "wheelchair":
            aid_path = f"{character['path']}/wheelchair"
            wheelchair = UsdGeom.Cube.Define(self.stage, aid_path)
            wheelchair.GetExtentAttr().Set([
                (-0.3, -0.3, 0),
                (0.3, 0.3, 0.5)
            ])
            wheelchair.GetDisplayColorAttr().Set([(0.2, 0.2, 0.2)])
            character["mobility_aid"] = "wheelchair"
            
        elif mobility == "walker":
            aid_path = f"{character['path']}/walker"
            walker = UsdGeom.Cube.Define(self.stage, aid_path)
            walker.GetExtentAttr().Set([
                (-0.3, -0.2, 0),
                (0.3, 0.2, 0.8)
            ])
            walker.GetDisplayColorAttr().Set([(0.5, 0.5, 0.5)])
            character["mobility_aid"] = "walker"
    
    def update_all_characters(self, delta_time: float):
        """
        更新所有人物
        
        Args:
            delta_time: 時間間隔（秒）
        """
        # 更新模擬時間
        self.current_time += timedelta(seconds=delta_time * 60)  # 假設1秒=1分鐘
        
        for char_id, character in self.characters.items():
            self.update_character(char_id, delta_time)
    
    def update_character(self, char_id: str, delta_time: float):
        """
        更新單一人物
        
        Args:
            char_id: 人物ID
            delta_time: 時間間隔
        """
        character = self.characters.get(char_id)
        if not character:
            return
        
        # 更新活動
        self._update_activity(character)
        
        # 更新移動
        self._update_movement(character, delta_time)
        
        # 更新狀態
        self._update_state(character, delta_time)
        
        # 更新互動
        self._check_interactions(character)
        
        # 更新動畫（如果有的話）
        self._update_animation(character)
    
    def _update_activity(self, character: Dict):
        """根據時程表更新當前活動"""
        current_time_str = self.current_time.strftime("%H:%M")
        schedule = character["schedule"]
        
        for activity in schedule:
            activity_time = activity["time"]
            if activity_time <= current_time_str:
                # 檢查是否在活動時間內
                duration = activity.get("duration", 30)
                end_time = (
                    datetime.strptime(activity_time, "%H:%M") + 
                    timedelta(minutes=duration)
                ).strftime("%H:%M")
                
                if current_time_str <= end_time:
                    if character["current_activity"] != activity:
                        # 切換活動
                        character["current_activity"] = activity
                        character["target_position"] = self._get_activity_location(
                            activity["location"]
                        )
                        logger.debug(f"{character['id']} 開始活動：{activity['activity']}")
                    break
    
    def _get_activity_location(self, location_name: str) -> List[float]:
        """取得活動地點座標"""
        locations = {
            # 1F 長照中心
            "activity_room": [0, 0, 0],
            "rest_area": [5, 0, 0],
            "dining": [10, 0, 0],
            "garden": [15, 10, 0],
            "rehab_room": [-5, 0, 0],
            "dementia_area": [-10, 0, 0],
            
            # 2F 托嬰中心
            "play_area": [0, 0, 3.5],
            "nap_room": [5, 0, 3.5],
            "classroom": [-5, 0, 3.5],
            "outdoor": [10, 10, 3.5],
            
            # 4F 青少年中心
            "basketball_court": [0, 0, 10.5],
            "study_room": [8, 5, 10.5],
            "social_area": [0, 8, 10.5],
            
            # 通用
            "lobby": [0, -10, 0],
            "nurse_station": [0, -5, 0]
        }
        
        base_position = locations.get(location_name, [0, 0, 0])
        
        # 加入一些隨機偏移避免重疊
        return [
            base_position[0] + random.uniform(-2, 2),
            base_position[1] + random.uniform(-2, 2),
            base_position[2]
        ]
    
    def _update_movement(self, character: Dict, delta_time: float):
        """更新人物移動"""
        if not character.get("target_position"):
            return
        
        current_pos = np.array(character["position"])
        target_pos = np.array(character["target_position"])
        
        # 計算方向
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction[:2])  # 只考慮水平距離
        
        if distance > 0.5:  # 還沒到達目標
            # 正規化方向
            direction = direction / np.linalg.norm(direction)
            
            # 根據人物類型和狀態決定速度
            speed = self._get_movement_speed(character)
            
            # 更新位置
            new_position = current_pos + direction * speed * delta_time
            character["position"] = new_position.tolist()
            
            # 更新朝向
            character["orientation"] = np.arctan2(direction[1], direction[0])
            
            # 更新 Prim 位置
            self._set_character_position(character["path"], new_position.tolist())
        else:
            # 到達目標
            character["target_position"] = None
    
    def _get_movement_speed(self, character: Dict) -> float:
        """取得移動速度"""
        base_speeds = {
            CharacterType.ELDERLY: 0.8,
            CharacterType.TODDLER: 0.5,
            CharacterType.YOUTH: 1.5,
            CharacterType.CAREGIVER: 1.2
        }
        
        base_speed = base_speeds.get(character["type"], 1.0)
        
        # 根據狀態調整
        if character["type"] == CharacterType.ELDERLY:
            mobility = character["attributes"].get("mobility", "independent")
            if mobility == "wheelchair":
                base_speed *= 1.2
            elif mobility == "walker":
                base_speed *= 0.6
        
        # 根據能量調整
        energy = character["state"]["energy"]
        base_speed *= (0.5 + 0.5 * energy)
        
        return base_speed
    
    def _update_state(self, character: Dict, delta_time: float):
        """更新人物狀態"""
        state = character["state"]
        
        # 能量消耗
        activity = character.get("current_activity")
        if activity:
            activity_type = activity.get("activity") if isinstance(activity, dict) else activity
            if activity_type in [ActivityType.SPORTS, ActivityType.EXERCISE]:
                state["energy"] -= 0.02 * delta_time
            elif activity_type in [ActivityType.RESTING, ActivityType.SLEEPING]:
                state["energy"] += 0.03 * delta_time
            else:
                state["energy"] -= 0.01 * delta_time
        
        # 飢餓增加
        state["hunger"] += 0.005 * delta_time
        if activity and activity_type == ActivityType.EATING:
            state["hunger"] = max(0, state["hunger"] - 0.5)
        
        # 社交需求
        if len(self._get_nearby_characters(character)) > 0:
            state["social_need"] = max(0, state["social_need"] - 0.01 * delta_time)
        else:
            state["social_need"] = min(1, state["social_need"] + 0.005 * delta_time)
        
        # 限制狀態值範圍
        for key in state:
            state[key] = max(0, min(1, state[key]))
    
    def _check_interactions(self, character: Dict):
        """檢查並處理人物互動"""
        nearby_characters = self._get_nearby_characters(character)
        
        for other_char in nearby_characters:
            # 檢查互動條件
            if self._should_interact(character, other_char):
                self._perform_interaction(character, other_char)
    
    def _get_nearby_characters(self, character: Dict, radius: float = 3.0) -> List[Dict]:
        """取得附近的人物"""
        nearby = []
        char_pos = np.array(character["position"])
        
        for other_id, other_char in self.characters.items():
            if other_id == character["id"]:
                continue
            
            other_pos = np.array(other_char["position"])
            distance = np.linalg.norm(char_pos - other_pos)
            
            if distance < radius:
                nearby.append(other_char)
        
        return nearby
    
    def _should_interact(self, char1: Dict, char2: Dict) -> bool:
        """判斷是否應該互動"""
        # 檢查是否在同一樓層
        if abs(char1["position"][2] - char2["position"][2]) > 0.5:
            return False
        
        # 檢查社交偏好
        if char1["attributes"]["social_preference"] == "solitary":
            return random.random() < 0.1
        
        # 檢查活動類型
        activity1 = char1.get("current_activity")
        activity2 = char2.get("current_activity")
        
        if activity1 and activity2:
            activity_type1 = activity1.get("activity") if isinstance(activity1, dict) else activity1
            activity_type2 = activity2.get("activity") if isinstance(activity2, dict) else activity2
            
            if activity_type1 == activity_type2 == ActivityType.SOCIALIZING:
                return random.random() < 0.8
        
        return random.random() < 0.2
    
    def _perform_interaction(self, char1: Dict, char2: Dict):
        """執行互動"""
        interaction = {
            "time": self.current_time,
            "partner": char2["id"],
            "type": "social",
            "duration": random.randint(30, 180)  # 秒
        }
        
        char1["interaction_history"].append(interaction)
        
        # 更新狀態
        char1["state"]["social_need"] = max(0, char1["state"]["social_need"] - 0.1)
        char1["state"]["happiness"] = min(1, char1["state"]["happiness"] + 0.05)
        
        logger.debug(f"{char1['id']} 與 {char2['id']} 互動")
    
    def _update_animation(self, character: Dict):
        """更新人物動畫（簡化版）"""
        # 在實際應用中，這裡會更新骨骼動畫
        # 目前只是簡單的位置更新
        pass
    
    def _set_character_position(self, char_path: str, position: List[float]):
        """設定人物位置"""
        char_prim = self.stage.GetPrimAtPath(char_path)
        if char_prim:
            # 考慮人物高度偏移
            adjusted_position = position.copy()
            adjusted_position[2] += 0.8  # 人物中心高度
            
            UsdGeom.XformCommonAPI(char_prim).SetTranslate(adjusted_position)
    
    def simulate_emergency(self, emergency_type: str, location: List[float]):
        """
        模擬緊急情況
        
        Args:
            emergency_type: 緊急情況類型
            location: 發生位置
        """
        logger.warning(f"緊急情況：{emergency_type} 在 {location}")
        
        # 找出所有照護人員
        caregivers = [
            char for char in self.characters.values()
            if char["type"] == CharacterType.CAREGIVER
        ]
        
        # 派遣最近的照護人員
        if caregivers:
            nearest = min(
                caregivers,
                key=lambda c: np.linalg.norm(
                    np.array(c["position"]) - np.array(location)
                )
            )
            
            nearest["current_activity"] = {
                "activity": ActivityType.EMERGENCY,
                "location": "emergency_site"
            }
            nearest["target_position"] = location
            
            logger.info(f"派遣 {nearest['id']} 處理緊急情況")
    
    def get_character_statistics(self) -> Dict:
        """取得人物統計資料"""
        stats = {
            "total": len(self.characters),
            "by_type": {},
            "by_floor": {},
            "by_activity": {},
            "average_happiness": 0,
            "average_energy": 0
        }
        
        happiness_sum = 0
        energy_sum = 0
        
        for character in self.characters.values():
            # 按類型統計
            char_type = character["type"].value
            if char_type not in stats["by_type"]:
                stats["by_type"][char_type] = 0
            stats["by_type"][char_type] += 1
            
            # 按樓層統計
            floor = self._get_floor_from_position(character["position"])
            if floor not in stats["by_floor"]:
                stats["by_floor"][floor] = 0
            stats["by_floor"][floor] += 1
            
            # 按活動統計
            activity = character.get("current_activity")
            if activity:
                activity_type = activity.get("activity") if isinstance(activity, dict) else activity
                activity_name = activity_type.value if hasattr(activity_type, 'value') else str(activity_type)
                if activity_name not in stats["by_activity"]:
                    stats["by_activity"][activity_name] = 0
                stats["by_activity"][activity_name] += 1
            
            # 累計狀態值
            happiness_sum += character["state"]["happiness"]
            energy_sum += character["state"]["energy"]
        
        if len(self.characters) > 0:
            stats["average_happiness"] = happiness_sum / len(self.characters)
            stats["average_energy"] = energy_sum / len(self.characters)
        
        return stats
    
    def _get_floor_from_position(self, position: List[float]) -> str:
        """根據位置判斷樓層"""
        z = position[2]
        
        if z < -1:
            return "B1"
        elif z < 2:
            return "1F"
        elif z < 5:
            return "2F"
        elif z < 9:
            return "3F"
        else:
            return "4F"


if __name__ == "__main__":
    """測試人物模擬系統"""
    # 初始化 Omniverse
    import omni.usd
    from omni.isaac.kit import SimulationApp
    
    # 建立模擬應用
    app = SimulationApp({"headless": False})
    
    # 建立人物模擬
    sim = CharacterSimulation()
    
    # 生成所有人物
    sim.spawn_all_characters()
    
    # 取得統計資料
    stats = sim.get_character_statistics()
    print(f"人物統計：{stats}")
    
    # 模擬主迴圈
    dt = 0.1  # 時間步長
    steps = 0
    
    while app.is_running() and steps < 1000:
        # 更新所有人物
        sim.update_all_characters(dt)
        
        # 每100步輸出一次統計
        if steps % 100 == 0:
            stats = sim.get_character_statistics()
            print(f"Step {steps}: {stats}")
        
        app.update()
        steps += 1
    
    app.close()
