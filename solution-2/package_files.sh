#!/bin/bash
# 赤土崎全齡社福樞紐 NVIDIA Isaac Sim 專案 - 檔案打包腳本
# 用途：一鍵打包所有生成的檔案
# 日期：2025年11月23日

echo "======================================"
echo "赤土崎全齡社福樞紐 3D 模擬專案"
echo "檔案打包工具 v1.0"
echo "======================================"
echo ""

# 設定專案名稱和時間戳記
PROJECT_NAME="easterlin-hsinchu-isaac-sim"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
ARCHIVE_NAME="${PROJECT_NAME}_${TIMESTAMP}"

# 建立暫時目錄
echo "📁 建立打包目錄..."
mkdir -p /tmp/${ARCHIVE_NAME}
mkdir -p /tmp/${ARCHIVE_NAME}/scripts
mkdir -p /tmp/${ARCHIVE_NAME}/docs

# 複製所有檔案
echo "📋 複製檔案中..."

# 文件類檔案
echo "  ➜ 複製文件檔案..."
cp /mnt/user-data/outputs/isaac_sim_building_implementation_guide.md /tmp/${ARCHIVE_NAME}/docs/ 2>/dev/null || echo "    ⚠ 實作指南未找到"
cp /mnt/user-data/outputs/README.md /tmp/${ARCHIVE_NAME}/ 2>/dev/null || echo "    ⚠ README未找到"
cp /mnt/user-data/outputs/FINAL_PROJECT_SUMMARY.md /tmp/${ARCHIVE_NAME}/docs/ 2>/dev/null || echo "    ⚠ 專案總結未找到"
cp /mnt/user-data/outputs/ALL_FILES_LIST.md /tmp/${ARCHIVE_NAME}/docs/ 2>/dev/null || echo "    ⚠ 檔案列表未找到"
cp /mnt/user-data/outputs/ULTIMATE_COMPLETE_REVIEW.md /tmp/${ARCHIVE_NAME}/docs/ 2>/dev/null || echo "    ⚠ 完整回顧未找到"

# 配置檔案
echo "  ➜ 複製配置檔案..."
cp /mnt/user-data/outputs/building_config.yaml /tmp/${ARCHIVE_NAME}/ 2>/dev/null || echo "    ⚠ 建築配置未找到"
cp /mnt/user-data/outputs/requirements.txt /tmp/${ARCHIVE_NAME}/ 2>/dev/null || echo "    ⚠ 套件清單未找到"

# 主程式檔案
echo "  ➜ 複製主程式..."
cp /mnt/user-data/outputs/main_simulation.py /tmp/${ARCHIVE_NAME}/ 2>/dev/null || echo "    ⚠ 主程式未找到"
cp /mnt/user-data/outputs/quickstart.py /tmp/${ARCHIVE_NAME}/ 2>/dev/null || echo "    ⚠ 快速啟動未找到"

# 模組檔案
echo "  ➜ 複製核心模組..."
cp /mnt/user-data/outputs/scripts/building_generator.py /tmp/${ARCHIVE_NAME}/scripts/ 2>/dev/null || echo "    ⚠ 建築生成器未找到"
cp /mnt/user-data/outputs/scripts/facility_placer.py /tmp/${ARCHIVE_NAME}/scripts/ 2>/dev/null || echo "    ⚠ 設施配置器未找到"
cp /mnt/user-data/outputs/scripts/character_simulation.py /tmp/${ARCHIVE_NAME}/scripts/ 2>/dev/null || echo "    ⚠ 人物模擬未找到"
cp /mnt/user-data/outputs/scripts/sensor_system.py /tmp/${ARCHIVE_NAME}/scripts/ 2>/dev/null || echo "    ⚠ 感測器系統未找到"

# 建立專案資訊檔
echo "📝 生成專案資訊..."
cat > /tmp/${ARCHIVE_NAME}/PROJECT_INFO.txt << EOF
================================================================================
赤土崎全齡社福樞紐 - NVIDIA Isaac Sim 3D 建築模擬專案
================================================================================

專案名稱: 赤土崎多功能館建築設計方案
生成日期: $(date +"%Y年%m月%d日 %H:%M:%S")
版本: 1.0.0
作者: Claude AI Assistant (Anthropic)

專案規格:
- 建築規模: 地下1層 + 地上4層 (B1+4F)
- 總面積: 3,100 m²
- 服務人數: 140-180人
- 技術平台: NVIDIA Isaac Sim / Omniverse
- 程式語言: Python 3.10+
- GPU需求: NVIDIA RTX 3060以上 (推薦RTX 5090)

檔案清單:
總檔案數: 13個
總大小: ~300KB
程式碼行數: ~3,500行

主要功能:
✅ 5層樓建築完整建模
✅ 180+ AI驅動人物模擬
✅ 80+ 智慧監控攝影機
✅ 跨齡互動活動設計
✅ 緊急應變系統
✅ 即時數據分析

使用說明:
1. 安裝 NVIDIA Isaac Sim
2. 安裝 Python 相依套件: pip install -r requirements.txt
3. 執行快速設置: python quickstart.py --setup
4. 啟動模擬: python main_simulation.py

技術支援:
- NVIDIA Isaac Sim 文件: https://docs.omniverse.nvidia.com/isaacsim
- USD 文件: https://openusd.org/
- Omniverse 文件: https://docs.omniverse.nvidia.com/

授權:
MIT License (開源專案)

================================================================================
EOF

# 建立快速安裝腳本
echo "🔧 生成安裝腳本..."
cat > /tmp/${ARCHIVE_NAME}/install.sh << 'EOF'
#!/bin/bash
echo "開始安裝赤土崎全齡社福樞紐模擬系統..."

# 檢查 Python 版本
python_version=$(python3 --version 2>&1 | grep -Po '(?<=Python )\d+\.\d+')
required_version="3.10"

if [ "$(printf '%s\n' "$required_version" "$python_version" | sort -V | head -n1)" != "$required_version" ]; then
    echo "錯誤: 需要 Python $required_version 或更高版本"
    exit 1
fi

# 建立專案目錄
mkdir -p assets/{architecture,furniture,equipment,characters}
mkdir -p scenes
mkdir -p saved_scenes
mkdir -p data/reports
mkdir -p logs

# 安裝相依套件
pip install -r requirements.txt

# 檢查 GPU
nvidia-smi > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "警告: 未檢測到 NVIDIA GPU"
fi

echo "安裝完成！請執行 python quickstart.py 開始使用"
EOF

chmod +x /tmp/${ARCHIVE_NAME}/install.sh

# 統計檔案
echo ""
echo "📊 檔案統計..."
FILE_COUNT=$(find /tmp/${ARCHIVE_NAME} -type f | wc -l)
TOTAL_SIZE=$(du -sh /tmp/${ARCHIVE_NAME} | cut -f1)
echo "  總檔案數: ${FILE_COUNT} 個"
echo "  總大小: ${TOTAL_SIZE}"

# 打包檔案
echo ""
echo "📦 打包檔案..."

# 建立 tar.gz 檔案
cd /tmp
tar -czf ${ARCHIVE_NAME}.tar.gz ${ARCHIVE_NAME}/
mv ${ARCHIVE_NAME}.tar.gz /mnt/user-data/outputs/

# 建立 zip 檔案（Windows 相容）
zip -r ${ARCHIVE_NAME}.zip ${ARCHIVE_NAME}/ > /dev/null 2>&1
mv ${ARCHIVE_NAME}.zip /mnt/user-data/outputs/

# 清理暫時檔案
rm -rf /tmp/${ARCHIVE_NAME}

echo ""
echo "✅ 打包完成！"
echo ""
echo "檔案位置:"
echo "  📦 TAR.GZ: /mnt/user-data/outputs/${ARCHIVE_NAME}.tar.gz"
echo "  📦 ZIP: /mnt/user-data/outputs/${ARCHIVE_NAME}.zip"
echo ""
echo "======================================"
echo "感謝使用！祝專案順利！"
echo "======================================"
