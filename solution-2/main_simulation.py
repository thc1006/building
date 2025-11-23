#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - NVIDIA Isaac Sim 3D 建築模擬
主程式檔案
Date: 2025-11-23
"""

import os
import sys
import argparse
import logging
from datetime import datetime
from pathlib import Path

# Isaac Sim imports
try:
    from omni.isaac.kit import SimulationApp
except ImportError:
    print("錯誤：請確保已安裝 NVIDIA Isaac Sim")
    print("請透過 Omniverse Launcher 安裝 Isaac Sim")
    sys.exit(1)

# 設定日誌
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('simulation.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class EasterlinBuildingSimulation:
    """赤土崎社福設施 3D 模擬主類別"""
    
    def __init__(self, config_path="configs/building_config.yaml", headless=False):
        """
        初始化模擬環境
        
        Args:
            config_path: 配置檔案路徑
            headless: 是否以無頭模式執行（不顯示 GUI）
        """
        logger.info("初始化 Isaac Sim 應用程式...")
        
        # 配置 Isaac Sim
        self.simulation_app = SimulationApp({
            "headless": headless,
            "renderer": "RayTracedLighting",
            "width": 1920,
            "height": 1080,
            "window_width": 1920,
            "window_height": 1080,
            "display_options": 3094,  # 顯示所有視覺化選項
        })
        
        self.config_path = config_path
        self.stage = None
        self.timeline = None
        self.components = {}
        
        # 初始化完成後載入核心組件
        self._initialize_core_components()
    
    def _initialize_core_components(self):
        """初始化核心組件"""
        logger.info("載入核心組件...")
        
        import omni
        from pxr import Usd, UsdGeom, Gf
        from omni.isaac.core import World
        from omni.isaac.core.utils.stage import add_reference_to_stage
        
        # 建立世界
        self.world = World(stage_units_in_meters=1.0)
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        
        # 設定舞台
        self._setup_stage()
        
        # 載入各模組
        self._load_modules()
    
    def _setup_stage(self):
        """設定舞台基本參數"""
        from pxr import UsdGeom
        
        logger.info("設定舞台參數...")
        
        # 設定舞台單位（公尺）
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        
        # 新增光源
        self._add_lighting()
        
        # 新增地面
        self._add_ground_plane()
        
        # 新增天空
        self._add_sky()
    
    def _add_lighting(self):
        """新增場景光源"""
        from pxr import UsdLux
        
        # 新增穹頂光
        dome_light = UsdLux.DomeLight.Define(self.stage, "/World/DomeLight")
        dome_light.GetIntensityAttr().Set(1000.0)
        
        # 新增太陽光（模擬日光）
        distant_light = UsdLux.DistantLight.Define(self.stage, "/World/SunLight")
        distant_light.GetIntensityAttr().Set(3000.0)
        distant_light.GetAngleAttr().Set(0.53)
        
        # 設定太陽角度（上午光線）
        from pxr import Gf
        distant_light.AddRotateOp().Set(Gf.Vec3f(-45, 30, 0))
    
    def _add_ground_plane(self):
        """新增地面"""
        from omni.isaac.core.objects import GroundPlane
        
        self.ground_plane = GroundPlane(
            prim_path="/World/GroundPlane",
            name="ground_plane",
            size=100,
            color=(0.5, 0.5, 0.5)
        )
    
    def _add_sky(self):
        """新增天空環境"""
        import omni.isaac.core.utils.prims as prims_utils
        
        # 使用預設的 HDRI 天空
        sky_path = "omniverse://localhost/NVIDIA/Assets/Skies/Clear/noon_grass_4k.hdr"
        
        # 這裡可以新增 HDRI 天空圖像
        # 需要額外的資源檔案
    
    def _load_modules(self):
        """載入各功能模組"""
        logger.info("載入功能模組...")
        
        try:
            # 載入建築生成器
            from scripts.building_generator import BuildingGenerator
            self.components['building'] = BuildingGenerator()
            
            # 載入設施配置器
            from scripts.facility_placer import FacilityPlacer
            self.components['facilities'] = FacilityPlacer(self.config_path)
            
            # 載入角色模擬
            from scripts.character_simulation import CharacterSimulation
            self.components['characters'] = CharacterSimulation()
            
            # 載入感測器系統
            from scripts.sensor_system import SensorSystem
            self.components['sensors'] = SensorSystem()
            
            # 載入 AI 行為系統
            from scripts.ai_behavior_system import AIBehaviorSystem
            self.components['ai'] = AIBehaviorSystem()
            
            # 載入監控面板
            from scripts.visualization_dashboard import MonitoringDashboard
            self.components['dashboard'] = MonitoringDashboard()
            
        except ImportError as e:
            logger.warning(f"某些模組尚未實作：{e}")
            logger.info("使用簡化版本繼續...")
            
            # 載入簡化版本
            self._load_simplified_building()
    
    def _load_simplified_building(self):
        """載入簡化的建築模型（用於測試）"""
        from pxr import UsdGeom, Gf
        
        logger.info("建立簡化建築模型...")
        
        # 建築參數
        building_specs = {
            "B1": {"height": 3.0, "size": [30, 20], "z": -3.0, "color": (0.5, 0.5, 0.5)},
            "1F": {"height": 3.5, "size": [40, 20], "z": 0, "color": (0.8, 0.7, 0.6)},
            "2F": {"height": 3.5, "size": [35, 20], "z": 3.5, "color": (0.9, 0.8, 0.7)},
            "3F": {"height": 3.5, "size": [25, 20], "z": 7.0, "color": (0.8, 0.8, 0.7)},
            "4F": {"height": 4.0, "size": [25, 20], "z": 10.5, "color": (0.7, 0.8, 0.9)}
        }
        
        building_root = UsdGeom.Xform.Define(self.stage, "/World/Building")
        
        for floor_name, specs in building_specs.items():
            self._create_floor(floor_name, specs, building_root.GetPath())
    
    def _create_floor(self, name, specs, parent_path):
        """建立單一樓層"""
        from pxr import UsdGeom, Gf
        import omni.isaac.core.utils.prims as prims_utils
        
        floor_path = f"{parent_path}/{name}"
        floor_xform = UsdGeom.Xform.Define(self.stage, floor_path)
        
        # 建立樓板（使用 Cube 代替）
        floor_plate = UsdGeom.Cube.Define(self.stage, f"{floor_path}/floor_plate")
        floor_plate.GetExtentAttr().Set([
            (-specs["size"][0]/2, -specs["size"][1]/2, -0.15),
            (specs["size"][0]/2, specs["size"][1]/2, 0.15)
        ])
        
        # 設定位置
        floor_plate.AddTranslateOp().Set(Gf.Vec3f(0, 0, specs["z"]))
        
        # 建立牆壁
        wall_thickness = 0.2
        wall_height = specs["height"]
        
        # 四面牆
        walls = [
            ("north", [0, specs["size"][1]/2, specs["z"] + wall_height/2], 
             [specs["size"][0], wall_thickness, wall_height]),
            ("south", [0, -specs["size"][1]/2, specs["z"] + wall_height/2],
             [specs["size"][0], wall_thickness, wall_height]),
            ("east", [specs["size"][0]/2, 0, specs["z"] + wall_height/2],
             [wall_thickness, specs["size"][1], wall_height]),
            ("west", [-specs["size"][0]/2, 0, specs["z"] + wall_height/2],
             [wall_thickness, specs["size"][1], wall_height])
        ]
        
        for wall_name, pos, size in walls:
            wall_path = f"{floor_path}/walls/{wall_name}"
            wall = UsdGeom.Cube.Define(self.stage, wall_path)
            wall.GetExtentAttr().Set([
                (-size[0]/2, -size[1]/2, -size[2]/2),
                (size[0]/2, size[1]/2, size[2]/2)
            ])
            wall.AddTranslateOp().Set(Gf.Vec3f(*pos))
        
        # 新增樓層標籤
        self._add_floor_label(floor_path, name, specs)
    
    def _add_floor_label(self, floor_path, name, specs):
        """新增樓層標示（用於識別）"""
        # 這裡可以新增 3D 文字或其他視覺標示
        logger.info(f"建立樓層：{name}")
    
    def create_building(self):
        """建立完整建築"""
        logger.info("開始建立建築物...")
        
        if 'building' in self.components:
            # 使用完整的建築生成器
            self.components['building'].generate_building()
        else:
            # 使用簡化版本
            logger.info("使用簡化建築模型")
    
    def place_facilities(self):
        """配置設施"""
        logger.info("配置設施與家具...")
        
        if 'facilities' in self.components:
            floors = ["b1_parking", "floor_1_elderly", "floor_2_nursery", 
                     "floor_3_family", "floor_4_youth"]
            for floor in floors:
                self.components['facilities'].place_floor_facilities(floor)
        else:
            logger.info("跳過設施配置（模組未載入）")
    
    def spawn_characters(self):
        """生成人物"""
        logger.info("生成虛擬人物...")
        
        if 'characters' in self.components:
            self.components['characters'].setup_characters()
        else:
            logger.info("跳過人物生成（模組未載入）")
    
    def setup_sensors(self):
        """設定感測器"""
        logger.info("配置感測器系統...")
        
        if 'sensors' in self.components:
            self.components['sensors'].setup_all_sensors()
        else:
            logger.info("跳過感測器設定（模組未載入）")
    
    def initialize_ai(self):
        """初始化 AI 系統"""
        logger.info("啟動 AI 行為系統...")
        
        if 'ai' in self.components:
            self.components['ai'].setup_behaviors()
        else:
            logger.info("跳過 AI 系統（模組未載入）")
    
    def open_dashboard(self):
        """開啟監控面板"""
        logger.info("開啟監控面板...")
        
        if 'dashboard' in self.components:
            self.components['dashboard'].create_dashboard()
        else:
            logger.info("跳過監控面板（模組未載入）")
    
    def run_simulation(self, duration=None):
        """
        執行模擬
        
        Args:
            duration: 模擬時長（秒），None 表示持續執行
        """
        logger.info("開始執行模擬...")
        
        # 重置世界
        self.world.reset()
        
        # 開始時間
        start_time = datetime.now()
        frame_count = 0
        
        try:
            while self.simulation_app.is_running():
                # 更新世界
                self.world.step(render=True)
                
                # 更新各組件
                self._update_components()
                
                frame_count += 1
                
                # 檢查是否達到指定時長
                if duration:
                    elapsed = (datetime.now() - start_time).total_seconds()
                    if elapsed >= duration:
                        logger.info(f"達到指定模擬時長：{duration} 秒")
                        break
                
                # 每 100 幀記錄一次
                if frame_count % 100 == 0:
                    logger.info(f"已執行 {frame_count} 幀")
        
        except KeyboardInterrupt:
            logger.info("使用者中斷模擬")
        
        except Exception as e:
            logger.error(f"模擬過程中發生錯誤：{e}")
            raise
        
        finally:
            elapsed_time = (datetime.now() - start_time).total_seconds()
            logger.info(f"模擬結束 - 執行時間：{elapsed_time:.2f} 秒，總幀數：{frame_count}")
    
    def _update_components(self):
        """更新所有組件"""
        # 更新 AI 行為
        if 'ai' in self.components:
            # self.components['ai'].update_all_behaviors()
            pass
        
        # 更新感測器數據
        if 'sensors' in self.components:
            # sensor_data = self.components['sensors'].get_current_data()
            pass
        
        # 更新儀表板
        if 'dashboard' in self.components:
            # self.components['dashboard'].update_dashboard(sensor_data)
            pass
    
    def save_scene(self, filename=None):
        """儲存場景"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"easterlin_building_{timestamp}.usd"
        
        save_path = os.path.join("saved_scenes", filename)
        os.makedirs("saved_scenes", exist_ok=True)
        
        logger.info(f"儲存場景至：{save_path}")
        self.stage.Export(save_path)
        
        return save_path
    
    def load_scene(self, filepath):
        """載入場景"""
        logger.info(f"載入場景：{filepath}")
        
        import omni.usd
        omni.usd.get_context().open_stage(filepath)
        
        # 重新初始化組件
        self._initialize_core_components()
    
    def close(self):
        """關閉模擬"""
        logger.info("關閉模擬...")
        
        if self.simulation_app:
            self.simulation_app.close()


def main():
    """主函式"""
    # 解析命令列參數
    parser = argparse.ArgumentParser(
        description="赤土崎全齡社福樞紐 - NVIDIA Isaac Sim 3D 建築模擬"
    )
    parser.add_argument(
        "--config", 
        type=str, 
        default="configs/building_config.yaml",
        help="配置檔案路徑"
    )
    parser.add_argument(
        "--headless", 
        action="store_true",
        help="以無頭模式執行（不顯示 GUI）"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="模擬執行時長（秒）"
    )
    parser.add_argument(
        "--load-scene",
        type=str,
        default=None,
        help="載入已儲存的場景檔案"
    )
    parser.add_argument(
        "--save-scene",
        action="store_true",
        help="模擬結束後儲存場景"
    )
    
    args = parser.parse_args()
    
    # 顯示啟動訊息
    print("=" * 60)
    print("赤土崎全齡社福樞紐 - 3D 建築模擬系統")
    print("使用 NVIDIA Isaac Sim")
    print(f"日期：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)
    
    # 初始化模擬
    sim = None
    
    try:
        # 建立模擬實例
        sim = EasterlinBuildingSimulation(
            config_path=args.config,
            headless=args.headless
        )
        
        # 載入場景或建立新場景
        if args.load_scene:
            sim.load_scene(args.load_scene)
        else:
            # 建立建築
            sim.create_building()
            
            # 配置設施
            sim.place_facilities()
            
            # 生成人物
            sim.spawn_characters()
            
            # 設定感測器
            sim.setup_sensors()
            
            # 初始化 AI
            sim.initialize_ai()
        
        # 開啟監控面板（非無頭模式）
        if not args.headless:
            sim.open_dashboard()
        
        # 執行模擬
        sim.run_simulation(duration=args.duration)
        
        # 儲存場景
        if args.save_scene:
            saved_path = sim.save_scene()
            print(f"場景已儲存至：{saved_path}")
    
    except Exception as e:
        logger.error(f"執行錯誤：{e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        # 清理資源
        if sim:
            sim.close()
    
    print("\n模擬完成！")
    return 0


if __name__ == "__main__":
    sys.exit(main())
