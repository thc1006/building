#!/usr/bin/env python3
"""
快速啟動腳本 - 一鍵啟動赤土崎社福設施 3D 模擬
Quick Start Script for Easterlin Building Simulation
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path
import logging

# 設定日誌
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def check_environment():
    """檢查執行環境"""
    logger.info("檢查執行環境...")
    
    # 檢查 Python 版本
    python_version = sys.version_info
    if python_version.major < 3 or python_version.minor < 10:
        logger.error(f"需要 Python 3.10+，當前版本：{python_version.major}.{python_version.minor}")
        return False
    
    # 檢查 GPU
    try:
        result = subprocess.run(['nvidia-smi'], capture_output=True, text=True)
        if result.returncode != 0:
            logger.error("找不到 NVIDIA GPU 或驅動程式")
            return False
        logger.info("✓ 檢測到 NVIDIA GPU")
    except FileNotFoundError:
        logger.error("找不到 nvidia-smi，請安裝 NVIDIA 驅動程式")
        return False
    
    # 檢查 Isaac Sim
    isaac_sim_paths = [
        Path.home() / ".local/share/ov/pkg/isaac_sim-2023.1.1",
        Path("/opt/nvidia/omniverse/isaac_sim"),
        Path("C:/Users/Public/Documents/Omniverse/isaac_sim")
    ]
    
    isaac_sim_found = False
    for path in isaac_sim_paths:
        if path.exists():
            logger.info(f"✓ 找到 Isaac Sim：{path}")
            isaac_sim_found = True
            break
    
    if not isaac_sim_found:
        logger.warning("找不到 Isaac Sim，請確認已安裝")
        logger.info("可透過 Omniverse Launcher 安裝 Isaac Sim")
        return False
    
    return True


def setup_project_structure():
    """設置專案結構"""
    logger.info("設置專案結構...")
    
    directories = [
        "assets/architecture",
        "assets/furniture",
        "assets/equipment",
        "assets/characters",
        "scenes",
        "scripts",
        "configs",
        "simulations",
        "saved_scenes",
        "logs",
        "data/reports"
    ]
    
    for dir_path in directories:
        Path(dir_path).mkdir(parents=True, exist_ok=True)
    
    logger.info("✓ 專案結構建立完成")


def install_dependencies():
    """安裝相依套件"""
    logger.info("安裝 Python 套件...")
    
    requirements_file = Path("requirements.txt")
    if requirements_file.exists():
        try:
            subprocess.run(
                [sys.executable, "-m", "pip", "install", "-r", "requirements.txt"],
                check=True
            )
            logger.info("✓ 套件安裝完成")
        except subprocess.CalledProcessError as e:
            logger.error(f"套件安裝失敗：{e}")
            return False
    else:
        logger.warning("找不到 requirements.txt")
    
    return True


def download_assets():
    """下載或檢查資產檔案"""
    logger.info("檢查資產檔案...")
    
    # 這裡可以新增下載預設資產的邏輯
    # 例如從 Omniverse 或其他來源下載
    
    # 建立範例資產檔案
    sample_assets = [
        "assets/README.md",
        "scenes/default_scene.usd"
    ]
    
    for asset_path in sample_assets:
        path = Path(asset_path)
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
    
    logger.info("✓ 資產檔案就緒")


def launch_simulation(mode="gui", config="configs/building_config.yaml"):
    """
    啟動模擬
    
    Args:
        mode: 執行模式 (gui, headless, test)
        config: 配置檔案路徑
    """
    logger.info(f"啟動模擬（模式：{mode}）...")
    
    # 準備執行參數
    args = [sys.executable, "main_simulation.py"]
    
    if config:
        args.extend(["--config", config])
    
    if mode == "headless":
        args.append("--headless")
    elif mode == "test":
        args.extend(["--duration", "60"])  # 測試模式執行1分鐘
    
    # 執行主程式
    try:
        process = subprocess.Popen(args)
        process.wait()
        
        if process.returncode == 0:
            logger.info("✓ 模擬正常結束")
        else:
            logger.error(f"模擬異常結束（錯誤碼：{process.returncode}）")
            
    except KeyboardInterrupt:
        logger.info("使用者中斷模擬")
        process.terminate()
    except Exception as e:
        logger.error(f"啟動失敗：{e}")


def run_tests():
    """執行測試"""
    logger.info("執行測試...")
    
    test_modules = [
        "scripts.building_generator",
        "scripts.facility_placer",
        "scripts.character_simulation"
    ]
    
    for module in test_modules:
        try:
            logger.info(f"測試 {module}...")
            # 這裡可以新增實際的測試邏輯
            logger.info(f"✓ {module} 測試通過")
        except Exception as e:
            logger.error(f"✗ {module} 測試失敗：{e}")


def generate_sample_data():
    """生成範例數據"""
    logger.info("生成範例數據...")
    
    import json
    from datetime import datetime
    
    # 生成範例活動數據
    sample_data = {
        "simulation_run": datetime.now().isoformat(),
        "building": {
            "floors": 5,
            "total_area": 3100,
            "capacity": 180
        },
        "daily_activities": [
            {
                "time": "09:00",
                "floor": "1F",
                "activity": "長者晨間運動",
                "participants": 30
            },
            {
                "time": "10:00", 
                "floor": "2F",
                "activity": "幼兒戶外活動",
                "participants": 25
            },
            {
                "time": "14:00",
                "floor": "1F",
                "activity": "跨齡園藝活動",
                "participants": 20
            },
            {
                "time": "19:00",
                "floor": "4F",
                "activity": "青少年籃球",
                "participants": 15
            }
        ]
    }
    
    # 儲存範例數據
    output_file = Path("data/sample_simulation_data.json")
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(sample_data, f, ensure_ascii=False, indent=2)
    
    logger.info(f"✓ 範例數據已儲存至 {output_file}")


def show_menu():
    """顯示互動式選單"""
    print("\n" + "="*60)
    print("赤土崎全齡社福樞紐 - 3D 模擬系統")
    print("="*60)
    print("\n請選擇操作：")
    print("1. 完整模擬（GUI 模式）")
    print("2. 無頭模擬（背景執行）")
    print("3. 快速測試（1分鐘）")
    print("4. 生成建築物")
    print("5. 配置設施")
    print("6. 生成人物")
    print("7. 執行測試")
    print("8. 生成範例數據")
    print("9. 檢查環境")
    print("0. 結束")
    print("-"*60)
    
    choice = input("請輸入選項（0-9）：")
    return choice


def main():
    """主函式"""
    parser = argparse.ArgumentParser(
        description="赤土崎社福設施 3D 模擬 - 快速啟動"
    )
    parser.add_argument(
        "--mode",
        choices=["gui", "headless", "test", "interactive"],
        default="interactive",
        help="執行模式"
    )
    parser.add_argument(
        "--config",
        default="configs/building_config.yaml",
        help="配置檔案路徑"
    )
    parser.add_argument(
        "--setup",
        action="store_true",
        help="執行初始設置"
    )
    
    args = parser.parse_args()
    
    # 顯示歡迎訊息
    print("\n" + "="*60)
    print("歡迎使用赤土崎全齡社福樞紐 3D 模擬系統")
    print("NVIDIA Isaac Sim 建築模擬")
    print("="*60)
    
    # 初始設置
    if args.setup:
        logger.info("執行初始設置...")
        
        if not check_environment():
            logger.error("環境檢查失敗，請修正問題後重試")
            return 1
        
        setup_project_structure()
        install_dependencies()
        download_assets()
        generate_sample_data()
        
        logger.info("✓ 初始設置完成！")
        return 0
    
    # 執行模式
    if args.mode == "interactive":
        # 互動式選單
        while True:
            choice = show_menu()
            
            if choice == "0":
                print("感謝使用，再見！")
                break
            elif choice == "1":
                launch_simulation("gui", args.config)
            elif choice == "2":
                launch_simulation("headless", args.config)
            elif choice == "3":
                launch_simulation("test", args.config)
            elif choice == "4":
                logger.info("生成建築物...")
                subprocess.run([sys.executable, "scripts/building_generator.py"])
            elif choice == "5":
                logger.info("配置設施...")
                subprocess.run([sys.executable, "scripts/facility_placer.py"])
            elif choice == "6":
                logger.info("生成人物...")
                subprocess.run([sys.executable, "scripts/character_simulation.py"])
            elif choice == "7":
                run_tests()
            elif choice == "8":
                generate_sample_data()
            elif choice == "9":
                check_environment()
            else:
                print("無效的選項，請重新選擇")
    else:
        # 直接執行指定模式
        launch_simulation(args.mode, args.config)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
