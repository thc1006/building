#!/usr/bin/env python3
"""
赤土崎全齡社福樞紐 - Isaac Sim 3D 建築生成器

使用 NVIDIA Isaac Sim 5.0 基於 OpenUSD 生成建築物的 3D 場景
配合 Claude Code CLI 進行自動化建模

作者: Claude Code Assistant
日期: 2025-11-23
"""

import os
import json
from typing import Dict, List, Tuple
from dataclasses import dataclass, asdict

# Isaac Sim 相關導入（需要在 Isaac Sim 環境中運行）
try:
    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({"headless": False})
    
    import omni
    from pxr import Usd, UsdGeom, Gf, UsdPhysics, UsdShade
    from omni.isaac.core import World
    from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
    from omni.isaac.core.prims import XFormPrim
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    print("⚠️ Isaac Sim 未安裝或未在正確環境中運行")
    print("請在 Isaac Sim Python 環境中執行此腳本")
    ISAAC_SIM_AVAILABLE = False


@dataclass
class Room:
    """房間資料結構"""
    name: str
    area: float  # m²
    position: Tuple[float, float, float]  # (x, y, z)
    dimensions: Tuple[float, float, float]  # (length, width, height)
    room_type: str  # 房間類型
    equipment: List[str]  # 設備清單
    
@dataclass
class Floor:
    """樓層資料結構"""
    floor_id: str
    name: str
    total_area: float
    height: float
    rooms: List[Room]
    
@dataclass
class Building:
    """建築物資料結構"""
    name: str
    floors: List[Floor]
    total_area: float


class EasterlinBuildingGenerator:
    """赤土崎建築物 3D 生成器"""
    
    def __init__(self, output_dir: str = "./output"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # 建築資料
        self.building = self._initialize_building_data()
        
        # Isaac Sim 世界
        self.world = None
        self.stage = None
        
    def _initialize_building_data(self) -> Building:
        """
        初始化建築資料
        從 architectural-floor-plans-2025.md 提取的資訊
        """
        
        # B1 層定義
        b1_rooms = [
            Room(
                name="停車區域",
                area=450,
                position=(0, 0, -3.5),
                dimensions=(30, 15, 3.5),
                room_type="parking",
                equipment=["parking_spaces", "lighting", "cctv"]
            ),
            Room(
                name="空調機房",
                area=30,
                position=(20, 20, -3.5),
                dimensions=(6, 5, 3.5),
                room_type="hvac",
                equipment=["hvac_unit", "heat_pump"]
            ),
            Room(
                name="配電室",
                area=20,
                position=(20, 26, -3.5),
                dimensions=(5, 4, 3.5),
                room_type="electrical",
                equipment=["main_panel", "ups"]
            ),
        ]
        
        # 1F 層定義
        f1_rooms = [
            Room(
                name="失智專區-安靜活動室",
                area=80,
                position=(0, 0, 0),
                dimensions=(10, 8, 3.5),
                room_type="activity",
                equipment=["adjustable_tables", "therapy_materials", "piano"]
            ),
            Room(
                name="失智專區-感官刺激室",
                area=60,
                position=(11, 0, 0),
                dimensions=(8, 7.5, 3.5),
                room_type="sensory",
                equipment=["fiber_optics", "bubble_tubes", "aroma_diffuser"]
            ),
            Room(
                name="共用餐廳",
                area=120,
                position=(0, 15, 0),
                dimensions=(12, 10, 3.5),
                room_type="dining",
                equipment=["dining_tables", "wheelchair_accessible_tables", "warming_carts"]
            ),
            Room(
                name="復健訓練室",
                area=80,
                position=(20, 0, 0),
                dimensions=(10, 8, 3.5),
                room_type="rehabilitation",
                equipment=["treadmill", "exercise_bike", "parallel_bars"]
            ),
        ]
        
        # 2F 層定義
        f2_rooms = [
            Room(
                name="嬰兒遊戲區",
                area=80,
                position=(0, 0, 3.5),
                dimensions=(10, 8, 3.5),
                room_type="infant_play",
                equipment=["crawling_mats", "sensory_toys", "safety_mirrors"]
            ),
            Room(
                name="嬰兒午睡室",
                area=60,
                position=(11, 0, 3.5),
                dimensions=(8, 7.5, 3.5),
                room_type="infant_sleep",
                equipment=["cribs", "white_noise", "blackout_curtains"]
            ),
            Room(
                name="幼兒遊戲區",
                area=120,
                position=(0, 10, 3.5),
                dimensions=(12, 10, 3.5),
                room_type="toddler_play",
                equipment=["building_blocks", "play_kitchen", "climbing_frame"]
            ),
            Room(
                name="幼兒餐廳",
                area=60,
                position=(20, 0, 3.5),
                dimensions=(8, 7.5, 3.5),
                room_type="toddler_dining",
                equipment=["child_tables", "high_chairs", "microwave"]
            ),
        ]
        
        # 3F 層定義
        f3_rooms = [
            Room(
                name="個別諮商室",
                area=60,
                position=(0, 0, 7),
                dimensions=(15, 4, 3.5),
                room_type="counseling",
                equipment=["sofas", "sound_machines", "emergency_buttons"]
            ),
            Room(
                name="多功能教室",
                area=100,
                position=(0, 10, 7),
                dimensions=(12.5, 8, 3.5),
                room_type="classroom",
                equipment=["projector", "sound_system", "stackable_chairs"]
            ),
            Room(
                name="親子烹飪教室",
                area=50,
                position=(15, 0, 7),
                dimensions=(8, 6.25, 3.5),
                room_type="cooking",
                equipment=["island_counter", "induction_cooktop", "oven"]
            ),
            Room(
                name="社區共餐廚房",
                area=60,
                position=(15, 10, 7),
                dimensions=(8, 7.5, 3.5),
                room_type="commercial_kitchen",
                equipment=["wok_station", "rice_cooker", "dishwasher"]
            ),
        ]
        
        # 4F 層定義
        f4_rooms = [
            Room(
                name="室內籃球場",
                area=150,
                position=(0, 0, 10.5),
                dimensions=(15, 10, 6),
                room_type="basketball",
                equipment=["basketball_hoops", "sport_flooring", "led_lights"]
            ),
            Room(
                name="舞蹈韻律教室",
                area=50,
                position=(16, 0, 10.5),
                dimensions=(8, 6.25, 3.5),
                room_type="dance",
                equipment=["mirror_wall", "ballet_barre", "sound_system"]
            ),
            Room(
                name="自習室",
                area=60,
                position=(0, 11, 10.5),
                dimensions=(10, 6, 3.5),
                room_type="study",
                equipment=["individual_desks", "desk_lamps", "bookshelves"]
            ),
            Room(
                name="電腦教室",
                area=50,
                position=(11, 11, 10.5),
                dimensions=(8, 6.25, 3.5),
                room_type="computer_lab",
                equipment=["computers", "monitors", "projector"]
            ),
            Room(
                name="創客空間",
                area=40,
                position=(20, 0, 10.5),
                dimensions=(8, 5, 3.5),
                room_type="makerspace",
                equipment=["3d_printer", "laser_cutter", "arduino_kits"]
            ),
        ]
        
        # 建立樓層
        floors = [
            Floor("B1", "停車場與設備層", 600, 3.5, b1_rooms),
            Floor("1F", "長照日照中心", 800, 3.5, f1_rooms),
            Floor("2F", "公共托嬰中心", 700, 3.5, f2_rooms),
            Floor("3F", "家庭支持服務中心", 500, 3.5, f3_rooms),
            Floor("4F", "青少年活動中心", 500, 6.0, f4_rooms),  # 4F 較高（籃球場）
        ]
        
        return Building(
            name="赤土崎全齡社福樞紐",
            floors=floors,
            total_area=3100
        )
    
    def setup_isaac_sim_world(self):
        """設置 Isaac Sim 世界環境"""
        if not ISAAC_SIM_AVAILABLE:
            print("❌ Isaac Sim 不可用，無法建立 3D 場景")
            return False
            
        # 建立世界
        self.world = World(stage_units_in_meters=1.0)
        self.stage = omni.usd.get_context().get_stage()
        
        # 設置場景
        self._setup_scene()
        
        print(f"✅ Isaac Sim 世界已建立")
        return True
    
    def _setup_scene(self):
        """設置場景基本元素"""
        # 建立地面
        ground_path = "/World/Ground"
        ground = VisualCuboid(
            prim_path=ground_path,
            position=[0, 0, -0.5],
            size=100,
            color=[0.5, 0.5, 0.5]
        )
        
        # 建立燈光
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="DistantLight",
            prim_path="/World/Sun"
        )
        
    def generate_floor_3d(self, floor: Floor):
        """
        為單一樓層生成 3D 模型
        
        Args:
            floor: 樓層資料
        """
        if not ISAAC_SIM_AVAILABLE:
            print(f"⚠️ 跳過 {floor.floor_id} 的 3D 生成（Isaac Sim 不可用）")
            return
            
        print(f"\n📐 生成樓層: {floor.floor_id} - {floor.name}")
        
        floor_path = f"/World/Building/{floor.floor_id}"
        
        # 建立樓層容器
        floor_xform = XFormPrim(prim_path=floor_path)
        
        # 為每個房間生成 3D
        for room in floor.rooms:
            self._create_room_3d(floor_path, room)
            
        print(f"✅ {floor.floor_id} 樓層生成完成")
    
    def _create_room_3d(self, floor_path: str, room: Room):
        """
        創建房間的 3D 模型
        
        Args:
            floor_path: 樓層路徑
            room: 房間資料
        """
        room_path = f"{floor_path}/{room.name.replace(' ', '_')}"
        
        # 建立房間牆壁
        wall_thickness = 0.2
        length, width, height = room.dimensions
        
        # 地板
        floor_cube = VisualCuboid(
            prim_path=f"{room_path}/Floor",
            position=[room.position[0], room.position[1], room.position[2] - height/2],
            size=[length, width, 0.1],
            color=[0.9, 0.9, 0.9]
        )
        
        # 四面牆
        walls = [
            # 前牆
            {
                "path": f"{room_path}/Wall_Front",
                "position": [room.position[0] - length/2, room.position[1], room.position[2]],
                "size": [wall_thickness, width, height]
            },
            # 後牆
            {
                "path": f"{room_path}/Wall_Back",
                "position": [room.position[0] + length/2, room.position[1], room.position[2]],
                "size": [wall_thickness, width, height]
            },
            # 左牆
            {
                "path": f"{room_path}/Wall_Left",
                "position": [room.position[0], room.position[1] - width/2, room.position[2]],
                "size": [length, wall_thickness, height]
            },
            # 右牆
            {
                "path": f"{room_path}/Wall_Right",
                "position": [room.position[0], room.position[1] + width/2, room.position[2]],
                "size": [length, wall_thickness, height]
            },
        ]
        
        for wall in walls:
            VisualCuboid(
                prim_path=wall["path"],
                position=wall["position"],
                size=wall["size"],
                color=[0.95, 0.95, 0.9]
            )
        
        # 天花板
        ceiling_cube = VisualCuboid(
            prim_path=f"{room_path}/Ceiling",
            position=[room.position[0], room.position[1], room.position[2] + height/2],
            size=[length, width, 0.1],
            color=[0.98, 0.98, 0.98]
        )
        
        # 添加設備（簡化為標記點）
        self._add_equipment_markers(room_path, room)
        
        print(f"  ✓ 房間已建立: {room.name} ({room.area} m²)")
    
    def _add_equipment_markers(self, room_path: str, room: Room):
        """
        添加設備標記
        
        Args:
            room_path: 房間路徑
            room: 房間資料
        """
        equipment_colors = {
            "wheelchair": [0.3, 0.5, 0.8],
            "table": [0.6, 0.4, 0.2],
            "chair": [0.7, 0.5, 0.3],
            "bed": [0.8, 0.6, 0.4],
            "equipment": [0.5, 0.5, 0.5],
        }
        
        for i, equipment in enumerate(room.equipment):
            equipment_path = f"{room_path}/Equipment_{i}_{equipment}"
            
            # 判斷設備類型選擇顏色
            color = [0.5, 0.5, 0.5]  # 預設灰色
            for key, eq_color in equipment_colors.items():
                if key in equipment.lower():
                    color = eq_color
                    break
            
            # 建立設備標記（小方塊）
            offset_x = (i % 3) * 1.5
            offset_y = (i // 3) * 1.5
            
            VisualCuboid(
                prim_path=equipment_path,
                position=[
                    room.position[0] + offset_x - 2,
                    room.position[1] + offset_y - 2,
                    room.position[2] - room.dimensions[2]/2 + 0.5
                ],
                size=[0.5, 0.5, 0.5],
                color=color
            )
    
    def generate_all_floors(self):
        """生成所有樓層的 3D 模型"""
        print("\n" + "="*60)
        print(f"🏗️ 開始生成: {self.building.name}")
        print(f"總面積: {self.building.total_area} m²")
        print(f"樓層數: {len(self.building.floors)}")
        print("="*60)
        
        for floor in self.building.floors:
            self.generate_floor_3d(floor)
        
        print("\n" + "="*60)
        print("✅ 所有樓層生成完成！")
        print("="*60)
    
    def export_usd(self, filename: str):
        """
        匯出 USD 場景檔案
        
        Args:
            filename: 輸出檔案名稱
        """
        if not ISAAC_SIM_AVAILABLE or not self.stage:
            print("⚠️ 無法匯出 USD（Isaac Sim 不可用或場景未建立）")
            return
            
        output_path = os.path.join(self.output_dir, filename)
        self.stage.Export(output_path)
        print(f"\n💾 USD 場景已匯出: {output_path}")
    
    def export_json(self, filename: str):
        """
        匯出建築資料為 JSON
        
        Args:
            filename: 輸出檔案名稱
        """
        output_path = os.path.join(self.output_dir, filename)
        
        building_dict = {
            "name": self.building.name,
            "total_area": self.building.total_area,
            "floors": [
                {
                    "floor_id": floor.floor_id,
                    "name": floor.name,
                    "total_area": floor.total_area,
                    "height": floor.height,
                    "rooms": [
                        {
                            "name": room.name,
                            "area": room.area,
                            "position": room.position,
                            "dimensions": room.dimensions,
                            "room_type": room.room_type,
                            "equipment": room.equipment
                        }
                        for room in floor.rooms
                    ]
                }
                for floor in self.building.floors
            ]
        }
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(building_dict, f, ensure_ascii=False, indent=2)
        
        print(f"💾 建築資料已匯出: {output_path}")
    
    def generate_floor_report(self, floor: Floor) -> str:
        """
        生成樓層報告
        
        Args:
            floor: 樓層資料
            
        Returns:
            報告文字
        """
        report = f"\n{'='*60}\n"
        report += f"樓層報告: {floor.floor_id} - {floor.name}\n"
        report += f"{'='*60}\n"
        report += f"總面積: {floor.total_area} m²\n"
        report += f"樓高: {floor.height} m\n"
        report += f"房間數: {len(floor.rooms)}\n\n"
        
        report += "房間清單:\n"
        report += "-" * 60 + "\n"
        
        for i, room in enumerate(floor.rooms, 1):
            report += f"{i}. {room.name}\n"
            report += f"   面積: {room.area} m²\n"
            report += f"   類型: {room.room_type}\n"
            report += f"   位置: {room.position}\n"
            report += f"   尺寸: {room.dimensions}\n"
            report += f"   設備: {', '.join(room.equipment)}\n\n"
        
        return report
    
    def run(self):
        """主要執行流程"""
        print("\n" + "="*60)
        print("🚀 赤土崎全齡社福樞紐 - 3D 建築生成器")
        print("="*60)
        
        # 匯出 JSON 資料
        self.export_json("easterlin_building_data.json")
        
        # 生成樓層報告
        for floor in self.building.floors:
            report = self.generate_floor_report(floor)
            print(report)
            
            # 儲存報告
            report_path = os.path.join(self.output_dir, f"{floor.floor_id}_report.txt")
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write(report)
        
        # 如果 Isaac Sim 可用，生成 3D 場景
        if ISAAC_SIM_AVAILABLE:
            if self.setup_isaac_sim_world():
                self.generate_all_floors()
                self.export_usd("easterlin_building.usd")
                
                print("\n" + "="*60)
                print("🎮 Isaac Sim 場景已準備完成")
                print("你可以在 Isaac Sim 中開啟 .usd 檔案查看 3D 模型")
                print("="*60)
        else:
            print("\n" + "="*60)
            print("ℹ️ 在沒有 Isaac Sim 的環境下運行")
            print("已生成建築資料 JSON 和報告")
            print("請在 Isaac Sim 環境中重新執行以生成 3D 場景")
            print("="*60)


def main():
    """主函數"""
    generator = EasterlinBuildingGenerator(output_dir="./easterlin_3d_output")
    generator.run()
    
    if ISAAC_SIM_AVAILABLE:
        # 保持 Isaac Sim 視窗開啟
        print("\n按 Ctrl+C 結束...")
        while simulation_app.is_running():
            simulation_app.update()
        
        simulation_app.close()


if __name__ == "__main__":
    main()
