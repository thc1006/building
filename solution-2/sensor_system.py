#!/usr/bin/env python3
"""
感測器系統 - 建築物監控與數據收集
Sensor System for Building Monitoring
"""

import omni
from omni.isaac.sensor import Camera, ContactSensor
from omni.isaac.core.utils.prims import create_prim
from pxr import Gf, UsdGeom, UsdPhysics, Sdf
import numpy as np
import cv2
from datetime import datetime
from typing import Dict, List, Tuple, Optional
from enum import Enum
import logging
import json

logger = logging.getLogger(__name__)


class SensorType(Enum):
    """感測器類型枚舉"""
    CAMERA = "camera"
    CONTACT = "contact"
    MOTION = "motion"
    TEMPERATURE = "temperature"
    HUMIDITY = "humidity"
    CO2 = "co2"
    NOISE = "noise"
    SMOKE = "smoke"
    EMERGENCY_BUTTON = "emergency_button"


class AlertLevel(Enum):
    """警報等級枚舉"""
    INFO = "info"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class SensorSystem:
    """感測器系統主類別"""
    
    def __init__(self, config_path: str = "configs/building_config.yaml"):
        """
        初始化感測器系統
        
        Args:
            config_path: 配置檔案路徑
        """
        self.stage = omni.usd.get_context().get_stage()
        self.sensors = {}
        self.sensor_data = {}
        self.alerts = []
        self.data_history = []
        
        # 載入配置
        self.config = self._load_config(config_path)
        
        # 初始化感測器類型
        self.camera_sensors = {}
        self.contact_sensors = {}
        self.environmental_sensors = {}
        self.emergency_sensors = {}
        
        logger.info("感測器系統初始化完成")
    
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
            "sensors": {
                "cameras": {
                    "total_count": 80,
                    "resolution": "1920x1080",
                    "fps": 30
                },
                "environmental": {
                    "temperature": {"range": [18, 28], "unit": "celsius"},
                    "humidity": {"range": [40, 70], "unit": "percent"},
                    "co2": {"range": [400, 1000], "unit": "ppm"},
                    "noise": {"range": [30, 85], "unit": "decibel"}
                },
                "safety": {
                    "smoke_detectors": {"interval": "15 sqm"},
                    "emergency_buttons": {"interval": "10m"}
                }
            }
        }
    
    def setup_all_sensors(self):
        """配置所有感測器"""
        logger.info("開始配置所有感測器...")
        
        # 配置監視攝影機
        self._setup_cameras()
        
        # 配置接觸感測器
        self._setup_contact_sensors()
        
        # 配置環境感測器
        self._setup_environmental_sensors()
        
        # 配置安全感測器
        self._setup_safety_sensors()
        
        # 配置緊急按鈕
        self._setup_emergency_buttons()
        
        logger.info(f"感測器配置完成，共配置 {len(self.sensors)} 個感測器")
    
    def _setup_cameras(self):
        """配置監視攝影機"""
        camera_locations = {
            # B1 停車場
            "b1_entrance": {"position": [-15, 0, -2], "rotation": [0, 45, 0], "fov": 90},
            "b1_parking_north": {"position": [0, 10, -2], "rotation": [0, 0, 0], "fov": 120},
            "b1_parking_south": {"position": [0, -10, -2], "rotation": [0, 180, 0], "fov": 120},
            "b1_elevator": {"position": [-5, -8, -2], "rotation": [0, 135, 0], "fov": 90},
            
            # 1F 長照中心
            "1f_entrance": {"position": [0, -10, 2], "rotation": [0, 0, 0], "fov": 90},
            "1f_reception": {"position": [0, -8, 2], "rotation": [0, 180, 0], "fov": 90},
            "1f_dementia_corridor": {"position": [-10, 0, 2], "rotation": [0, 90, 0], "fov": 120},
            "1f_activity_room": {"position": [0, 0, 2], "rotation": [-30, 0, 0], "fov": 120},
            "1f_dining": {"position": [10, 0, 2], "rotation": [-30, 45, 0], "fov": 90},
            "1f_garden": {"position": [15, 10, 2], "rotation": [0, -45, 0], "fov": 120},
            "1f_rehab": {"position": [-5, 0, 2], "rotation": [0, 90, 0], "fov": 90},
            
            # 2F 托嬰中心
            "2f_entrance": {"position": [0, -10, 5.5], "rotation": [0, 0, 0], "fov": 90},
            "2f_infant_room": {"position": [-8, 0, 5.5], "rotation": [-30, 45, 0], "fov": 120},
            "2f_toddler_room": {"position": [5, 0, 5.5], "rotation": [-30, -45, 0], "fov": 120},
            "2f_nap_room": {"position": [5, 5, 5.5], "rotation": [-45, 0, 0], "fov": 90},
            "2f_outdoor_play": {"position": [12, 8, 5.5], "rotation": [0, -90, 0], "fov": 120},
            
            # 3F 家庭支持
            "3f_multipurpose": {"position": [0, 0, 9], "rotation": [-30, 0, 0], "fov": 120},
            "3f_kitchen": {"position": [8, 0, 9], "rotation": [0, -90, 0], "fov": 90},
            "3f_consultation": {"position": [-8, 0, 9], "rotation": [0, 90, 0], "fov": 90},
            
            # 4F 青少年中心
            "4f_basketball": {"position": [0, 0, 13], "rotation": [-45, 0, 0], "fov": 120},
            "4f_study_room": {"position": [8, 5, 13], "rotation": [-30, -45, 0], "fov": 90},
            "4f_computer_lab": {"position": [8, -5, 13], "rotation": [-30, 45, 0], "fov": 90},
            "4f_social_area": {"position": [0, 8, 13], "rotation": [0, 180, 0], "fov": 120}
        }
        
        for camera_name, spec in camera_locations.items():
            self._create_camera(camera_name, spec)
    
    def _create_camera(self, name: str, spec: Dict):
        """
        建立單一攝影機
        
        Args:
            name: 攝影機名稱
            spec: 攝影機規格
        """
        camera_path = f"/World/Sensors/Cameras/{name}"
        
        # 建立攝影機 Prim
        camera_prim = create_prim(
            prim_path=camera_path,
            prim_type="Camera"
        )
        
        # 設定攝影機參數
        camera = Camera(
            prim_path=camera_path,
            frequency=self.config["sensors"]["cameras"].get("fps", 30),
            resolution=(1920, 1080)
        )
        
        # 設定位置和旋轉
        camera.set_world_pose(
            position=spec["position"],
            orientation=euler_to_quat(spec.get("rotation", [0, 0, 0]))
        )
        
        # 設定視野
        camera.set_horizontal_aperture(20.955)  # 標準值
        camera.set_focal_length(18.147)  # 標準值
        
        # 啟用深度輸出（用於 3D 分析）
        camera.set_enable_depth_output(True)
        
        # 儲存攝影機
        self.camera_sensors[name] = {
            "camera": camera,
            "spec": spec,
            "last_frame": None,
            "motion_detected": False,
            "people_count": 0
        }
        
        self.sensors[name] = {
            "type": SensorType.CAMERA,
            "path": camera_path,
            "active": True
        }
        
        logger.debug(f"建立攝影機：{name}")
    
    def _setup_contact_sensors(self):
        """配置接觸感測器"""
        contact_locations = {
            # 門感測器
            "1f_main_door": {"path": "/World/Building/floor_1_elderly/entrance_door", "type": "door"},
            "1f_emergency_exit": {"path": "/World/Building/floor_1_elderly/emergency_exit", "type": "door"},
            "2f_main_door": {"path": "/World/Building/floor_2_nursery/entrance_door", "type": "door"},
            "3f_main_door": {"path": "/World/Building/floor_3_family/entrance_door", "type": "door"},
            "4f_main_door": {"path": "/World/Building/floor_4_youth/entrance_door", "type": "door"},
            
            # 設備感測器
            "1f_wheelchair_1": {"path": "/World/Building/floor_1_elderly/wheelchair_1", "type": "equipment"},
            "1f_walker_1": {"path": "/World/Building/floor_1_elderly/walker_1", "type": "equipment"},
            "2f_crib_1": {"path": "/World/Building/floor_2_nursery/crib_1", "type": "equipment"},
            "4f_treadmill": {"path": "/World/Building/floor_4_youth/treadmill", "type": "equipment"},
            
            # 床位感測器
            "1f_recliner_1": {"path": "/World/Building/floor_1_elderly/recliner_1", "type": "bed"},
            "2f_nap_bed_1": {"path": "/World/Building/floor_2_nursery/nap_bed_1", "type": "bed"}
        }
        
        for sensor_name, spec in contact_locations.items():
            self._create_contact_sensor(sensor_name, spec)
    
    def _create_contact_sensor(self, name: str, spec: Dict):
        """建立接觸感測器"""
        sensor_path = f"{spec['path']}/contact_sensor"
        
        # 檢查物體是否存在
        parent_prim = self.stage.GetPrimAtPath(spec['path'])
        if not parent_prim:
            logger.debug(f"找不到物體：{spec['path']}")
            return
        
        # 建立感測器
        sensor = ContactSensor(
            prim_path=sensor_path,
            min_threshold=0.1,
            max_threshold=10000.0,
            radius=0.5
        )
        
        # 儲存感測器
        self.contact_sensors[name] = {
            "sensor": sensor,
            "spec": spec,
            "contact_count": 0,
            "last_contact_time": None,
            "in_use": False
        }
        
        self.sensors[name] = {
            "type": SensorType.CONTACT,
            "path": sensor_path,
            "active": True
        }
    
    def _setup_environmental_sensors(self):
        """配置環境感測器"""
        # 每層樓配置環境感測器
        floors = ["B1", "1F", "2F", "3F", "4F"]
        
        for floor in floors:
            # 溫度感測器
            self._create_environmental_sensor(
                f"{floor}_temperature",
                SensorType.TEMPERATURE,
                self._get_floor_position(floor)
            )
            
            # 濕度感測器
            self._create_environmental_sensor(
                f"{floor}_humidity",
                SensorType.HUMIDITY,
                self._get_floor_position(floor)
            )
            
            # CO2 感測器
            self._create_environmental_sensor(
                f"{floor}_co2",
                SensorType.CO2,
                self._get_floor_position(floor)
            )
            
            # 噪音感測器
            self._create_environmental_sensor(
                f"{floor}_noise",
                SensorType.NOISE,
                self._get_floor_position(floor)
            )
    
    def _create_environmental_sensor(self, name: str, sensor_type: SensorType,
                                    position: List[float]):
        """建立環境感測器"""
        sensor_path = f"/World/Sensors/Environmental/{name}"
        
        # 建立感測器 Prim（用小方塊表示）
        sensor_prim = UsdGeom.Cube.Define(self.stage, sensor_path)
        sensor_prim.GetExtentAttr().Set([
            (-0.1, -0.1, -0.1),
            (0.1, 0.1, 0.1)
        ])
        
        # 設定位置
        UsdGeom.XformCommonAPI(sensor_prim).SetTranslate(position)
        
        # 設定顏色（根據類型）
        colors = {
            SensorType.TEMPERATURE: [(1.0, 0.5, 0.0)],  # 橙色
            SensorType.HUMIDITY: [(0.0, 0.5, 1.0)],     # 藍色
            SensorType.CO2: [(0.5, 0.5, 0.5)],          # 灰色
            SensorType.NOISE: [(1.0, 1.0, 0.0)]         # 黃色
        }
        sensor_prim.GetDisplayColorAttr().Set(colors.get(sensor_type, [(1, 1, 1)]))
        
        # 儲存感測器
        self.environmental_sensors[name] = {
            "type": sensor_type,
            "position": position,
            "current_value": self._get_initial_value(sensor_type),
            "threshold": self._get_threshold(sensor_type),
            "unit": self._get_unit(sensor_type)
        }
        
        self.sensors[name] = {
            "type": sensor_type,
            "path": sensor_path,
            "active": True
        }
    
    def _setup_safety_sensors(self):
        """配置安全感測器"""
        # 煙霧偵測器
        smoke_positions = {
            "b1_smoke_1": [-10, 0, -2],
            "b1_smoke_2": [10, 0, -2],
            "1f_smoke_1": [-10, 0, 2],
            "1f_smoke_2": [0, 0, 2],
            "1f_smoke_3": [10, 0, 2],
            "2f_smoke_1": [-8, 0, 5.5],
            "2f_smoke_2": [5, 0, 5.5],
            "3f_smoke_1": [0, 0, 9],
            "4f_smoke_1": [0, 0, 13]
        }
        
        for name, position in smoke_positions.items():
            self._create_smoke_detector(name, position)
    
    def _create_smoke_detector(self, name: str, position: List[float]):
        """建立煙霧偵測器"""
        sensor_path = f"/World/Sensors/Safety/Smoke/{name}"
        
        # 建立偵測器 Prim
        detector = UsdGeom.Cylinder.Define(self.stage, sensor_path)
        detector.GetRadiusAttr().Set(0.08)
        detector.GetHeightAttr().Set(0.05)
        
        # 設定位置（天花板）
        ceiling_position = position.copy()
        ceiling_position[2] += 3.3  # 貼近天花板
        UsdGeom.XformCommonAPI(detector).SetTranslate(ceiling_position)
        
        # 設定白色
        detector.GetDisplayColorAttr().Set([(1.0, 1.0, 1.0)])
        
        # 儲存感測器
        self.sensors[name] = {
            "type": SensorType.SMOKE,
            "path": sensor_path,
            "active": True,
            "triggered": False
        }
    
    def _setup_emergency_buttons(self):
        """配置緊急按鈕"""
        button_positions = {
            "1f_emergency_1": [-10, -5, 1],
            "1f_emergency_2": [10, -5, 1],
            "1f_emergency_toilet": [0, 5, 1],
            "2f_emergency_1": [-8, -5, 4.5],
            "2f_emergency_2": [5, -5, 4.5],
            "3f_emergency": [0, -5, 8],
            "4f_emergency": [0, -5, 11.5]
        }
        
        for name, position in button_positions.items():
            self._create_emergency_button(name, position)
    
    def _create_emergency_button(self, name: str, position: List[float]):
        """建立緊急按鈕"""
        button_path = f"/World/Sensors/Emergency/{name}"
        
        # 建立按鈕 Prim
        button = UsdGeom.Cube.Define(self.stage, button_path)
        button.GetExtentAttr().Set([
            (-0.05, -0.02, -0.05),
            (0.05, 0.02, 0.05)
        ])
        
        # 設定位置（牆上）
        UsdGeom.XformCommonAPI(button).SetTranslate(position)
        
        # 設定紅色
        button.GetDisplayColorAttr().Set([(1.0, 0.0, 0.0)])
        
        # 儲存感測器
        self.emergency_sensors[name] = {
            "position": position,
            "pressed": False,
            "last_pressed_time": None
        }
        
        self.sensors[name] = {
            "type": SensorType.EMERGENCY_BUTTON,
            "path": button_path,
            "active": True
        }
    
    def _get_floor_position(self, floor: str) -> List[float]:
        """取得樓層中心位置"""
        positions = {
            "B1": [0, 0, -2],
            "1F": [0, 0, 2],
            "2F": [0, 0, 5.5],
            "3F": [0, 0, 9],
            "4F": [0, 0, 13]
        }
        return positions.get(floor, [0, 0, 0])
    
    def _get_initial_value(self, sensor_type: SensorType) -> float:
        """取得感測器初始值"""
        initial_values = {
            SensorType.TEMPERATURE: 22.0,
            SensorType.HUMIDITY: 55.0,
            SensorType.CO2: 500.0,
            SensorType.NOISE: 45.0
        }
        return initial_values.get(sensor_type, 0.0)
    
    def _get_threshold(self, sensor_type: SensorType) -> Dict:
        """取得感測器閾值"""
        config = self.config.get("sensors", {}).get("environmental", {})
        
        thresholds = {
            SensorType.TEMPERATURE: config.get("temperature", {}).get("range", [18, 28]),
            SensorType.HUMIDITY: config.get("humidity", {}).get("range", [40, 70]),
            SensorType.CO2: config.get("co2", {}).get("range", [400, 1000]),
            SensorType.NOISE: config.get("noise", {}).get("range", [30, 85])
        }
        
        return {"min": thresholds[sensor_type][0], "max": thresholds[sensor_type][1]}
    
    def _get_unit(self, sensor_type: SensorType) -> str:
        """取得感測器單位"""
        units = {
            SensorType.TEMPERATURE: "°C",
            SensorType.HUMIDITY: "%",
            SensorType.CO2: "ppm",
            SensorType.NOISE: "dB"
        }
        return units.get(sensor_type, "")
    
    def update_sensors(self, delta_time: float):
        """
        更新所有感測器數據
        
        Args:
            delta_time: 時間間隔
        """
        # 更新攝影機
        self._update_cameras()
        
        # 更新接觸感測器
        self._update_contact_sensors()
        
        # 更新環境感測器
        self._update_environmental_sensors(delta_time)
        
        # 檢查警報條件
        self._check_alert_conditions()
        
        # 記錄數據
        self._log_sensor_data()
    
    def _update_cameras(self):
        """更新攝影機數據"""
        for name, camera_data in self.camera_sensors.items():
            camera = camera_data["camera"]
            
            # 取得當前幀
            frame = camera.get_current_frame()
            
            if frame:
                # 儲存幀
                camera_data["last_frame"] = frame
                
                # 執行電腦視覺分析
                if "rgba" in frame:
                    image = frame["rgba"]
                    
                    # 動作偵測
                    camera_data["motion_detected"] = self._detect_motion(
                        name, image
                    )
                    
                    # 人數統計
                    camera_data["people_count"] = self._count_people(image)
                    
                    # 跌倒偵測（使用深度資訊）
                    if "depth" in frame:
                        self._detect_fall(name, frame["depth"])
    
    def _detect_motion(self, camera_name: str, current_frame: np.ndarray) -> bool:
        """
        動作偵測
        
        Args:
            camera_name: 攝影機名稱
            current_frame: 當前幀
            
        Returns:
            是否偵測到動作
        """
        # 簡化的動作偵測（實際應使用更複雜的算法）
        # 這裡只是示範
        
        if not hasattr(self, "_previous_frames"):
            self._previous_frames = {}
        
        if camera_name in self._previous_frames:
            prev_frame = self._previous_frames[camera_name]
            
            # 計算幀差
            diff = cv2.absdiff(current_frame, prev_frame)
            
            # 計算變化量
            motion_pixels = np.sum(diff > 30)
            total_pixels = diff.size
            
            motion_ratio = motion_pixels / total_pixels
            
            # 如果超過 5% 的像素有變化，判定為有動作
            motion_detected = motion_ratio > 0.05
        else:
            motion_detected = False
        
        self._previous_frames[camera_name] = current_frame.copy()
        
        return motion_detected
    
    def _count_people(self, image: np.ndarray) -> int:
        """
        計算人數（簡化版）
        
        Args:
            image: 影像
            
        Returns:
            偵測到的人數
        """
        # 實際應用中應使用 YOLO、OpenPose 等模型
        # 這裡只是隨機生成示範數據
        return np.random.poisson(5)  # 平均 5 人
    
    def _detect_fall(self, camera_name: str, depth_image: np.ndarray):
        """跌倒偵測"""
        # 簡化的跌倒偵測邏輯
        min_height = np.min(depth_image[depth_image > 0]) if depth_image.any() else float('inf')
        
        # 如果最低點低於 0.5 公尺，可能是跌倒
        if min_height < 0.5:
            self._trigger_alert(
                AlertLevel.CRITICAL,
                f"可能偵測到跌倒事件 - {camera_name}",
                {"camera": camera_name, "height": min_height}
            )
    
    def _update_contact_sensors(self):
        """更新接觸感測器"""
        for name, sensor_data in self.contact_sensors.items():
            sensor = sensor_data["sensor"]
            
            # 取得接觸資訊
            contacts = sensor.get_contact_forces()
            
            if contacts and len(contacts) > 0:
                sensor_data["contact_count"] = len(contacts)
                sensor_data["in_use"] = True
                sensor_data["last_contact_time"] = datetime.now()
            else:
                sensor_data["in_use"] = False
    
    def _update_environmental_sensors(self, delta_time: float):
        """更新環境感測器"""
        for name, sensor_data in self.environmental_sensors.items():
            sensor_type = sensor_data["type"]
            current_value = sensor_data["current_value"]
            
            # 模擬環境變化
            if sensor_type == SensorType.TEMPERATURE:
                # 溫度緩慢變化
                change = np.random.normal(0, 0.1) * delta_time
                new_value = current_value + change
                
            elif sensor_type == SensorType.HUMIDITY:
                # 濕度變化
                change = np.random.normal(0, 0.5) * delta_time
                new_value = current_value + change
                
            elif sensor_type == SensorType.CO2:
                # CO2 根據人數變化
                floor = name.split("_")[0]
                people_count = self._get_floor_occupancy(floor)
                base_co2 = 400 + people_count * 10
                new_value = base_co2 + np.random.normal(0, 50)
                
            elif sensor_type == SensorType.NOISE:
                # 噪音根據活動變化
                base_noise = 35
                if "4F" in name:  # 青少年中心較吵
                    base_noise = 65
                elif "2F" in name:  # 托嬰中心
                    base_noise = 50
                new_value = base_noise + np.random.normal(0, 10)
            else:
                new_value = current_value
            
            # 限制在合理範圍內
            threshold = sensor_data["threshold"]
            new_value = max(threshold["min"] - 5, min(threshold["max"] + 5, new_value))
            
            sensor_data["current_value"] = new_value
    
    def _get_floor_occupancy(self, floor: str) -> int:
        """取得樓層人數"""
        total = 0
        for camera_data in self.camera_sensors.values():
            if floor.lower() in camera_data.get("spec", {}).get("position", [0, 0, 0]):
                total += camera_data.get("people_count", 0)
        return total
    
    def _check_alert_conditions(self):
        """檢查警報條件"""
        # 檢查環境感測器
        for name, sensor_data in self.environmental_sensors.items():
            value = sensor_data["current_value"]
            threshold = sensor_data["threshold"]
            
            if value < threshold["min"] or value > threshold["max"]:
                level = AlertLevel.WARNING
                if abs(value - threshold["min"]) > 10 or abs(value - threshold["max"]) > 10:
                    level = AlertLevel.CRITICAL
                
                self._trigger_alert(
                    level,
                    f"{name} 超出正常範圍：{value:.1f}{sensor_data['unit']}",
                    {"sensor": name, "value": value, "threshold": threshold}
                )
        
        # 檢查緊急按鈕
        for name, button_data in self.emergency_sensors.items():
            if button_data["pressed"]:
                self._trigger_alert(
                    AlertLevel.EMERGENCY,
                    f"緊急按鈕被按下：{name}",
                    {"button": name, "position": button_data["position"]}
                )
                button_data["pressed"] = False  # 重置
    
    def _trigger_alert(self, level: AlertLevel, message: str, data: Dict):
        """
        觸發警報
        
        Args:
            level: 警報等級
            message: 警報訊息
            data: 相關數據
        """
        alert = {
            "timestamp": datetime.now().isoformat(),
            "level": level.value,
            "message": message,
            "data": data
        }
        
        self.alerts.append(alert)
        
        # 根據等級採取行動
        if level == AlertLevel.EMERGENCY:
            logger.critical(f"緊急警報：{message}")
            self._handle_emergency(data)
        elif level == AlertLevel.CRITICAL:
            logger.error(f"嚴重警報：{message}")
        elif level == AlertLevel.WARNING:
            logger.warning(f"警告：{message}")
        else:
            logger.info(f"資訊：{message}")
    
    def _handle_emergency(self, data: Dict):
        """處理緊急情況"""
        # 通知所有相關人員
        # 啟動緊急協議
        # 記錄事件
        pass
    
    def _log_sensor_data(self):
        """記錄感測器數據"""
        current_data = {
            "timestamp": datetime.now().isoformat(),
            "cameras": {
                name: {
                    "motion": data["motion_detected"],
                    "people": data["people_count"]
                }
                for name, data in self.camera_sensors.items()
            },
            "environmental": {
                name: {
                    "value": data["current_value"],
                    "unit": data["unit"]
                }
                for name, data in self.environmental_sensors.items()
            },
            "contacts": {
                name: {
                    "in_use": data["in_use"],
                    "count": data["contact_count"]
                }
                for name, data in self.contact_sensors.items()
            }
        }
        
        self.data_history.append(current_data)
        
        # 限制歷史記錄大小
        if len(self.data_history) > 10000:
            self.data_history = self.data_history[-5000:]
    
    def get_current_data(self) -> Dict:
        """取得當前感測器數據"""
        return {
            "floors": self._get_floor_status(),
            "alerts": self.alerts[-10:],  # 最近10個警報
            "environmental": self._get_environmental_summary(),
            "occupancy": self._get_occupancy_data()
        }
    
    def _get_floor_status(self) -> Dict:
        """取得各樓層狀態"""
        floors = ["B1", "1F", "2F", "3F", "4F"]
        status = {}
        
        for floor in floors:
            occupancy = self._get_floor_occupancy(floor)
            
            # 根據樓層決定容量
            capacities = {
                "B1": 30,
                "1F": 60,
                "2F": 50,
                "3F": 30,
                "4F": 40
            }
            
            status[floor] = {
                "current": occupancy,
                "capacity": capacities.get(floor, 50),
                "status": "normal"  # 可以根據條件改變
            }
        
        return status
    
    def _get_environmental_summary(self) -> Dict:
        """取得環境數據摘要"""
        summary = {}
        
        for sensor_type in [SensorType.TEMPERATURE, SensorType.HUMIDITY, 
                           SensorType.CO2, SensorType.NOISE]:
            values = []
            for name, data in self.environmental_sensors.items():
                if data["type"] == sensor_type:
                    values.append(data["current_value"])
            
            if values:
                summary[sensor_type.value] = {
                    "average": np.mean(values),
                    "min": np.min(values),
                    "max": np.max(values),
                    "unit": self._get_unit(sensor_type)
                }
        
        return summary
    
    def _get_occupancy_data(self) -> Dict:
        """取得佔用率數據"""
        total_people = sum(
            data["people_count"] 
            for data in self.camera_sensors.values()
        )
        
        return {
            "total": total_people,
            "by_floor": {
                floor: self._get_floor_occupancy(floor)
                for floor in ["B1", "1F", "2F", "3F", "4F"]
            }
        }
    
    def export_data(self, filepath: str, format: str = "json"):
        """
        匯出感測器數據
        
        Args:
            filepath: 檔案路徑
            format: 格式 (json, csv)
        """
        if format == "json":
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(self.data_history, f, ensure_ascii=False, indent=2)
        
        elif format == "csv":
            import pandas as pd
            
            # 轉換為 DataFrame
            records = []
            for data in self.data_history:
                record = {"timestamp": data["timestamp"]}
                
                # 扁平化數據
                for sensor_type, sensors in data.items():
                    if sensor_type != "timestamp":
                        for sensor_name, sensor_data in sensors.items():
                            for key, value in sensor_data.items():
                                column_name = f"{sensor_type}_{sensor_name}_{key}"
                                record[column_name] = value
                
                records.append(record)
            
            df = pd.DataFrame(records)
            df.to_csv(filepath, index=False)
        
        logger.info(f"數據已匯出至：{filepath}")
    
    def simulate_sensor_failure(self, sensor_name: str):
        """模擬感測器故障"""
        if sensor_name in self.sensors:
            self.sensors[sensor_name]["active"] = False
            self._trigger_alert(
                AlertLevel.WARNING,
                f"感測器故障：{sensor_name}",
                {"sensor": sensor_name}
            )
    
    def calibrate_sensor(self, sensor_name: str):
        """校準感測器"""
        if sensor_name in self.environmental_sensors:
            sensor = self.environmental_sensors[sensor_name]
            sensor["current_value"] = self._get_initial_value(sensor["type"])
            logger.info(f"感測器已校準：{sensor_name}")


def euler_to_quat(euler: List[float]) -> List[float]:
    """歐拉角轉四元數"""
    # 簡化版本，實際應使用正確的轉換公式
    return [0, 0, 0, 1]


if __name__ == "__main__":
    """測試感測器系統"""
    # 初始化 Omniverse
    import omni.usd
    from omni.isaac.kit import SimulationApp
    
    # 建立模擬應用
    app = SimulationApp({"headless": False})
    
    # 建立感測器系統
    sensor_system = SensorSystem()
    
    # 配置所有感測器
    sensor_system.setup_all_sensors()
    
    # 模擬主迴圈
    dt = 0.1
    steps = 0
    
    while app.is_running() and steps < 1000:
        # 更新感測器
        sensor_system.update_sensors(dt)
        
        # 每100步輸出一次數據
        if steps % 100 == 0:
            data = sensor_system.get_current_data()
            print(f"Step {steps}: 佔用率={data['occupancy']['total']}")
            print(f"  環境：{data['environmental']}")
            
            if data["alerts"]:
                print(f"  警報：{data['alerts'][-1]}")
        
        app.update()
        steps += 1
    
    # 匯出數據
    sensor_system.export_data("sensor_data.json")
    
    app.close()
