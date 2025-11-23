#!/bin/bash
# Easterlin Building 3D Generation - Claude Code Automation Script
# 
# 此腳本整合 Claude Code CLI 與 Isaac Sim 建築生成流程
# 使用方式: ./claude_code_workflow.sh [command]

set -e

# 顏色輸出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 專案設定
PROJECT_NAME="easterlin-building-3d"
PYTHON_VERSION="3.10"
ISAAC_SIM_VERSION="5.0.0"

# 目錄結構
PROJECT_DIR="$(pwd)/$PROJECT_NAME"
OUTPUT_DIR="$PROJECT_DIR/output"
SCRIPTS_DIR="$PROJECT_DIR/scripts"
DATA_DIR="$PROJECT_DIR/data"
DOCS_DIR="$PROJECT_DIR/docs"

# 函數: 印出帶顏色的訊息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 函數: 檢查必要工具
check_prerequisites() {
    print_info "檢查必要工具..."
    
    # 檢查 Python
    if ! command -v python3 &> /dev/null; then
        print_error "Python 3 未安裝"
        exit 1
    fi
    
    # 檢查 Claude Code CLI
    if ! command -v claude-code &> /dev/null; then
        print_warning "Claude Code CLI 未安裝"
        print_info "請參考: https://docs.claude.com/claude-code"
    fi
    
    # 檢查 Isaac Sim（可選）
    if [ -d "$HOME/.local/share/ov/pkg/isaac-sim-$ISAAC_SIM_VERSION" ]; then
        print_success "Isaac Sim $ISAAC_SIM_VERSION 已安裝"
        export ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac-sim-$ISAAC_SIM_VERSION"
    else
        print_warning "Isaac Sim 未安裝在預設路徑"
        print_info "如果已安裝，請設定 ISAAC_SIM_PATH 環境變數"
    fi
    
    print_success "前置檢查完成"
}

# 函數: 初始化專案結構
init_project() {
    print_info "初始化專案: $PROJECT_NAME"
    
    # 建立目錄結構
    mkdir -p "$OUTPUT_DIR"/{usd,json,reports,images}
    mkdir -p "$SCRIPTS_DIR"
    mkdir -p "$DATA_DIR"
    mkdir -p "$DOCS_DIR"
    
    # 複製主腳本
    cp isaac_sim_building_generator.py "$SCRIPTS_DIR/"
    
    # 建立 Python 虛擬環境
    print_info "建立 Python 虛擬環境..."
    python3 -m venv "$PROJECT_DIR/venv"
    
    # 啟動虛擬環境
    source "$PROJECT_DIR/venv/bin/activate"
    
    # 安裝相依套件
    print_info "安裝相依套件..."
    pip install --upgrade pip
    pip install numpy pyyaml trimesh pillow
    
    print_success "專案結構初始化完成"
    
    # 建立 README
    cat > "$PROJECT_DIR/README.md" << 'EOF'
# 赤土崎全齡社福樞紐 3D 建築生成專案

此專案使用 NVIDIA Isaac Sim 為赤土崎社福設施生成 3D 建築模型。

## 專案結構

```
easterlin-building-3d/
├── output/               # 輸出檔案
│   ├── usd/             # USD 3D 場景檔案
│   ├── json/            # 建築資料 JSON
│   ├── reports/         # 樓層報告
│   └── images/          # 渲染圖片
├── scripts/             # Python 腳本
├── data/                # 原始建築資料
├── docs/                # 文檔
└── venv/                # Python 虛擬環境
```

## 使用方式

### 方法 1: 使用 Claude Code CLI（推薦）

```bash
# 初始化專案
./claude_code_workflow.sh init

# 生成建築資料
./claude_code_workflow.sh generate-data

# 使用 Isaac Sim 生成 3D（需要 Isaac Sim 環境）
./claude_code_workflow.sh generate-3d

# 查看報告
./claude_code_workflow.sh report
```

### 方法 2: 直接執行 Python 腳本

```bash
# 啟動虛擬環境
source venv/bin/activate

# 執行生成器（不需要 Isaac Sim）
python scripts/isaac_sim_building_generator.py

# 在 Isaac Sim 環境中執行
$ISAAC_SIM_PATH/python.sh scripts/isaac_sim_building_generator.py
```

## 輸出檔案

- `easterlin_building_data.json`: 完整建築資料結構
- `easterlin_building.usd`: Universal Scene Description 3D 場景
- `{FLOOR_ID}_report.txt`: 各樓層詳細報告

## 系統需求

- Python 3.10+
- NVIDIA Isaac Sim 5.0+（3D 生成功能）
- CUDA-capable GPU（建議 RTX 5090）
- Ubuntu 22.04+ / Windows 11 with WSL2

## Claude Code CLI 整合

此專案設計為與 Claude Code CLI 無縫整合，可以透過對話方式調整建築參數：

```bash
claude-code chat "修改 2F 嬰兒室面積為 90 m²"
claude-code chat "增加 1F 復健設備"
claude-code chat "生成所有樓層的 3D 模型"
```

EOF
    
    print_success "README 已建立"
}

# 函數: 使用 Claude Code 生成建築資料
generate_data_with_claude() {
    print_info "使用 Claude Code 生成建築資料..."
    
    cd "$PROJECT_DIR"
    
    # 如果有 Claude Code CLI，使用它來處理
    if command -v claude-code &> /dev/null; then
        print_info "透過 Claude Code 提取建築資料..."
        
        claude-code chat "讀取 architectural-floor-plans-2025.md 並生成結構化的建築資料 JSON" \
            --output "$DATA_DIR/building_structure.json"
        
        print_success "建築資料已透過 Claude Code 生成"
    else
        # 直接執行 Python 腳本
        python3 "$SCRIPTS_DIR/isaac_sim_building_generator.py"
        
        # 移動輸出檔案
        mv easterlin_3d_output/* "$OUTPUT_DIR/" 2>/dev/null || true
        
        print_success "建築資料已生成"
    fi
}

# 函數: 生成 3D 模型（需要 Isaac Sim）
generate_3d_model() {
    print_info "生成 3D 模型..."
    
    if [ -z "$ISAAC_SIM_PATH" ]; then
        print_error "未找到 Isaac Sim，請設定 ISAAC_SIM_PATH"
        print_info "範例: export ISAAC_SIM_PATH=/path/to/isaac-sim"
        exit 1
    fi
    
    cd "$PROJECT_DIR"
    
    # 使用 Isaac Sim 的 Python 執行
    print_info "使用 Isaac Sim Python 環境執行生成器..."
    "$ISAAC_SIM_PATH/python.sh" "$SCRIPTS_DIR/isaac_sim_building_generator.py"
    
    # 移動輸出檔案到正確位置
    if [ -d "easterlin_3d_output" ]; then
        mv easterlin_3d_output/*.usd "$OUTPUT_DIR/usd/" 2>/dev/null || true
        mv easterlin_3d_output/*.json "$OUTPUT_DIR/json/" 2>/dev/null || true
        mv easterlin_3d_output/*.txt "$OUTPUT_DIR/reports/" 2>/dev/null || true
        rm -rf easterlin_3d_output
    fi
    
    print_success "3D 模型已生成"
    print_info "USD 檔案位置: $OUTPUT_DIR/usd/"
}

# 函數: 生成報告
generate_report() {
    print_info "生成樓層報告..."
    
    cd "$PROJECT_DIR"
    
    # 執行生成器（僅生成報告，不需要 Isaac Sim）
    python3 "$SCRIPTS_DIR/isaac_sim_building_generator.py"
    
    # 顯示報告摘要
    echo ""
    echo "=========================================="
    echo "樓層報告摘要"
    echo "=========================================="
    
    for report in "$OUTPUT_DIR"/reports/*.txt; do
        if [ -f "$report" ]; then
            echo ""
            cat "$report"
        fi
    done
    
    print_success "報告已生成"
}

# 函數: 使用 Claude Code 互動式調整
interactive_adjust() {
    print_info "啟動 Claude Code 互動式調整..."
    
    if ! command -v claude-code &> /dev/null; then
        print_error "Claude Code CLI 未安裝"
        exit 1
    fi
    
    cd "$PROJECT_DIR"
    
    print_info "你現在可以使用自然語言調整建築參數，例如："
    echo "  - '修改 2F 嬰兒室面積為 90 m²'"
    echo "  - '增加 1F 復健訓練室的設備'"
    echo "  - '調整 4F 籃球場高度為 7 公尺'"
    echo ""
    
    # 啟動 Claude Code 互動模式
    claude-code chat --project "$PROJECT_DIR"
}

# 函數: 清理輸出
clean() {
    print_info "清理輸出檔案..."
    
    rm -rf "$OUTPUT_DIR"/{usd,json,reports,images}/*
    
    print_success "輸出檔案已清理"
}

# 函數: 顯示專案狀態
show_status() {
    print_info "專案狀態"
    echo "=========================================="
    echo "專案目錄: $PROJECT_DIR"
    echo "Python 版本: $(python3 --version)"
    
    if [ -d "$PROJECT_DIR/venv" ]; then
        echo "虛擬環境: ✓ 已建立"
    else
        echo "虛擬環境: ✗ 未建立"
    fi
    
    if [ ! -z "$ISAAC_SIM_PATH" ]; then
        echo "Isaac Sim: ✓ $ISAAC_SIM_PATH"
    else
        echo "Isaac Sim: ✗ 未設定"
    fi
    
    echo ""
    echo "輸出檔案統計:"
    echo "  USD 模型: $(ls -1 "$OUTPUT_DIR/usd" 2>/dev/null | wc -l) 個"
    echo "  JSON 資料: $(ls -1 "$OUTPUT_DIR/json" 2>/dev/null | wc -l) 個"
    echo "  樓層報告: $(ls -1 "$OUTPUT_DIR/reports" 2>/dev/null | wc -l) 個"
    echo "=========================================="
}

# 函數: 顯示使用說明
show_help() {
    cat << EOF
赤土崎全齡社福樞紐 3D 建築生成 - Claude Code 自動化腳本

使用方式: $0 [command]

指令:
  init              初始化專案結構
  check             檢查系統需求
  generate-data     生成建築資料（不需要 Isaac Sim）
  generate-3d       生成 3D 模型（需要 Isaac Sim）
  report            生成樓層報告
  interactive       啟動 Claude Code 互動式調整
  status            顯示專案狀態
  clean             清理輸出檔案
  help              顯示此說明

範例:
  # 完整流程
  $0 init
  $0 generate-data
  $0 generate-3d
  
  # 使用 Claude Code 互動調整
  $0 interactive
  
  # 僅生成報告（不需要 Isaac Sim）
  $0 report

環境變數:
  ISAAC_SIM_PATH    Isaac Sim 安裝路徑
                    範例: export ISAAC_SIM_PATH=~/.local/share/ov/pkg/isaac-sim-5.0.0

EOF
}

# 主程式
main() {
    case "${1:-help}" in
        init)
            check_prerequisites
            init_project
            ;;
        check)
            check_prerequisites
            ;;
        generate-data)
            generate_data_with_claude
            ;;
        generate-3d)
            generate_3d_model
            ;;
        report)
            generate_report
            ;;
        interactive)
            interactive_adjust
            ;;
        status)
            show_status
            ;;
        clean)
            clean
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "未知指令: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# 執行主程式
main "$@"
