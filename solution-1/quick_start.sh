#!/bin/bash
# Easterlin Building 3D - 快速啟動腳本
# 下載所有檔案後，執行此腳本即可自動設置專案

set -e

echo "🏗️ 赤土崎全齡社福樞紐 3D 建築生成專案"
echo "=========================================="
echo ""

# 檢查當前目錄
CURRENT_DIR=$(pwd)
echo "📍 當前目錄: $CURRENT_DIR"
echo ""

# 詢問是否要在當前目錄建立專案
read -p "是否在當前目錄建立專案？(y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ 已取消"
    exit 1
fi

echo "📁 建立目錄結構..."
mkdir -p scripts output/{usd,json,reports,images} data docs

echo "📝 移動檔案到正確位置..."
# 如果檔案在當前目錄，移動它們
if [ -f "isaac_sim_building_generator.py" ]; then
    mv isaac_sim_building_generator.py scripts/
fi
if [ -f "claude_code_workflow.sh" ]; then
    mv claude_code_workflow.sh scripts/
    chmod +x scripts/claude_code_workflow.sh
fi
if [ -f "IMPLEMENTATION_GUIDE.md" ]; then
    mv IMPLEMENTATION_GUIDE.md docs/
fi
if [ -f "PROJECT_SUMMARY.md" ]; then
    mv PROJECT_SUMMARY.md docs/
fi
if [ -f "COMPLETE_PACKAGE.md" ]; then
    mv COMPLETE_PACKAGE.md docs/
fi

echo "🐍 建立 Python 虛擬環境..."
python3 -m venv venv

echo "📦 啟動虛擬環境並安裝套件..."
source venv/bin/activate
pip install --upgrade pip
pip install numpy pyyaml trimesh pillow

echo ""
echo "✅ 專案設置完成！"
echo "=========================================="
echo ""
echo "📂 目錄結構:"
tree -L 2 -I 'venv' || ls -R

echo ""
echo "🚀 下一步："
echo ""
echo "1️⃣  啟動虛擬環境:"
echo "   source venv/bin/activate"
echo ""
echo "2️⃣  生成建築資料（不需要 Isaac Sim）:"
echo "   python scripts/isaac_sim_building_generator.py"
echo ""
echo "3️⃣  查看結果:"
echo "   cat output/json/easterlin_building_data.json | python -m json.tool"
echo "   cat output/reports/1F_report.txt"
echo ""
echo "4️⃣  如果有 Isaac Sim，生成 3D:"
echo "   export ISAAC_SIM_PATH=\"\$HOME/.local/share/ov/pkg/isaac-sim-5.0.0\""
echo "   \$ISAAC_SIM_PATH/python.sh scripts/isaac_sim_building_generator.py"
echo ""
echo "📖 更多資訊請查看 README.md 和 docs/ 目錄"
echo ""
echo "🎉 祝你使用愉快！"
