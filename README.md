# 赤土崎全齡社福樞紐 - 3D 建築建模專案

使用 NVIDIA Isaac Sim 建立建築物 3D 模型

---

## 📋 專案概述

本專案旨在為「赤土崎全齡社福樞紐」建築設計方案建立完整的 3D 視覺化模型。

### 建築規格
- **樓層**: B1（地下1層）+ 4F（地上4層）
- **總面積**: 3,100 m²
- **建築高度**: 約 18-20 公尺
- **主要功能**:
  - B1: 停車場、設備層
  - 1F: 長照日照中心（800 m²）
  - 2F: 公共托嬰中心（700 m²）
  - 3F: 家庭支持服務中心（500 m²）
  - 4F: 青少年活動中心（500 m²）

---

## 🚀 快速開始

### 方式 1: 使用 Blender（推薦新手）

```bash
# 1. 安裝 Blender
sudo snap install blender --classic

# 2. 執行建模腳本
blender --background --python scripts/blender_generate_1F.py

# 3. 匯出 USD
# 開啟 Blender → File → Export → USD (.usdc)
```

### 方式 2: 使用 Isaac Sim（進階）

```bash
# 1. 安裝 Isaac Sim（透過 Omniverse Launcher）
# https://www.nvidia.com/en-us/omniverse/

# 2. 執行生成腳本
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh \
  --exec scripts/generate_floor_1F.py

# 3. 開啟場景
# Isaac Sim → File → Open → assets/floor_1F.usd
```

**詳細步驟請參閱 [QUICKSTART.md](./QUICKSTART.md)**

---

## 📁 專案結構

```
building/
├── CLAUDE.md                          # 建築設計詳細文檔
├── README.md                          # 本文件
├── QUICKSTART.md                      # 快速入門指南
├── isaac_sim_implementation_plan.md  # 完整實作計劃（60頁）
│
├── scripts/                           # Python 建模腳本
│   ├── blender_generate_1F.py        # ✅ Blender 1F 腳本
│   ├── generate_floor_1F.py          # ✅ Isaac Sim 1F 腳本
│   ├── blender_generate_2F.py        # ⏳ 待建立
│   ├── blender_generate_3F.py        # ⏳ 待建立
│   ├── blender_generate_4F.py        # ⏳ 待建立
│   └── blender_generate_B1.py        # ⏳ 待建立
│
├── assets/                            # USD 模型檔案
│   ├── floor_1F.usdc                 # ⏳ 待生成
│   ├── floor_2F.usdc                 # ⏳ 待生成
│   └── ...
│
├── scenes/                            # 完整場景
│   └── complete_building.usd         # ⏳ 待建立
│
├── renders/                           # 渲染輸出
│   ├── images/                       # 靜態圖片
│   └── videos/                       # 動畫影片
│
└── blender/                           # Blender 專案檔
    └── floor_1F.blend                # ⏳ 待儲存
```

---

## 🎯 實作路線圖

### Phase 1: 原型開發（1-2週）
- [x] 分析建築設計文檔
- [x] 建立專案結構
- [x] 撰寫 1F 建模腳本（Blender + Isaac Sim）
- [ ] 測試 Blender → USD → Isaac Sim 工作流程
- [ ] 調整材質、照明、攝影機

### Phase 2: 完整建模（2-3週）
- [ ] 建立 B1 停車場模型
- [ ] 建立 2F 托嬰中心模型
- [ ] 建立 3F 家庭支持中心模型
- [ ] 建立 4F 青少年中心模型
- [ ] 組裝完整 5 層建築場景

### Phase 3: 細節完善（1-2週）
- [ ] 添加家具與設備
- [ ] 優化材質與貼圖
- [ ] 添加室外景觀（庭園、停車場）
- [ ] 建立多個攝影機視角

### Phase 4: 互動與動畫（選配）
- [ ] 添加電梯動畫
- [ ] 添加門開關動畫
- [ ] 人員行走模擬（NVIDIA Character）
- [ ] 日夜光照變化

### Phase 5: 成果輸出（1週）
- [ ] 渲染高品質靜態圖（每層樓 3-5 張）
- [ ] 製作建築導覽影片（3-5 分鐘）
- [ ] 產出 VR 虛擬實境導覽（選配）

---

## 📊 技術規格

### 硬體需求
- **GPU**: NVIDIA RTX 3060 或更高（建議 RTX 4070+）
- **RAM**: 32GB 以上
- **儲存**: 150GB 可用空間
- **OS**: Ubuntu 20.04/22.04 或 Windows 10/11

### 軟體堆疊
- **NVIDIA Isaac Sim**: 2023.1.1+
- **Blender**: 3.6+
- **Python**: 3.10+
- **USD**: 23.11+

---

## 📚 文檔索引

| 文檔 | 用途 | 適合對象 |
|------|------|---------|
| [CLAUDE.md](./CLAUDE.md) | 建築設計詳細文檔 | 建築師、設計師 |
| [QUICKSTART.md](./QUICKSTART.md) | 快速入門指南 | 初學者 |
| [isaac_sim_implementation_plan.md](./isaac_sim_implementation_plan.md) | 完整實作計劃 | 開發者、技術人員 |
| [README.md](./README.md) | 專案總覽（本文件） | 所有人 |

---

## 🛠️ 開發工作流程

### 建議工作流程：Blender → USD → Isaac Sim

```mermaid
graph LR
    A[建築設計文檔<br/>CLAUDE.md] --> B[Blender 建模<br/>blender_generate_1F.py]
    B --> C[匯出 USD<br/>floor_1F.usdc]
    C --> D[Isaac Sim 組裝<br/>complete_building.usd]
    D --> E[調整材質/照明]
    E --> F[渲染輸出<br/>images/videos]

    style A fill:#E6E6FA
    style B fill:#90EE90
    style C fill:#FFD700
    style D fill:#FFB6C1
    style F fill:#87CEEB
```

**為什麼這樣做？**
1. ✅ Blender 建模工具更強大、直觀
2. ✅ USD 是中間格式，確保互通性
3. ✅ Isaac Sim 擅長場景組裝、物理模擬、渲染

---

## 🎨 視覺化範例

### 預期成果（參考）

**1F 長照日照中心 - 鳥瞰圖**:
```
預期呈現:
- 失智專區（左側，淺粉紅色地板）
- 一般日照區（中央，淺藍色地板）
- 共用餐廳（右側，淺黃色地板）
- 無障礙庭園（戶外，綠色植栽）
```

**完整建築 - 側視圖**:
```
預期呈現:
- 5 層樓層疊加（B1 至 4F）
- 各樓層高度差異
- 窗戶、陽台、庭園
```

---

## 📞 支援與協作

### 需要幫助？

1. **技術問題**: 查閱 [isaac_sim_implementation_plan.md](./isaac_sim_implementation_plan.md) 的「常見問題」章節
2. **Blender 教學**: [Blender 官方文檔](https://docs.blender.org/)
3. **Isaac Sim 教學**: [NVIDIA Omniverse 文檔](https://docs.omniverse.nvidia.com/isaacsim/latest/)

### 社群資源
- [NVIDIA Omniverse 論壇](https://forums.developer.nvidia.com/c/omniverse/)
- [Blender Artists 論壇](https://blenderartists.org/)
- [USD 論壇](https://groups.google.com/g/usd-interest)

---

## 📈 專案狀態

**更新日期**: 2025-11-23

| 項目 | 狀態 | 進度 |
|------|------|------|
| 專案規劃 | ✅ 完成 | 100% |
| 1F 腳本開發 | ✅ 完成 | 100% |
| 1F 模型生成 | ⏳ 進行中 | 0% |
| 其他樓層腳本 | ⏳ 待開始 | 0% |
| 完整場景組裝 | ⏳ 待開始 | 0% |
| 渲染輸出 | ⏳ 待開始 | 0% |

**總體進度**: 約 15% 完成

---

## 🌟 貢獻指南

### 如果你想參與此專案

1. **建模其他樓層**: 參考 `scripts/blender_generate_1F.py`，建立 2F、3F、4F、B1 的腳本
2. **添加家具庫**: 在 `assets/furniture/` 建立可重複使用的家具模型
3. **優化腳本**: 提升建模腳本的效率或功能
4. **建立教學**: 撰寫更多教學文檔或影片

### 編碼規範
- Python: 遵循 PEP 8
- 註解: 中文 + 英文（重要部分）
- 命名: 使用有意義的變數名稱

---

## 📄 授權

本專案為「赤土崎全齡社福樞紐」建築設計視覺化工具。

建築設計版權歸原設計單位所有。
3D 建模腳本與工具採用 MIT License。

---

## 🎯 下一步行動

### 立即可做
1. ✅ 閱讀 [QUICKSTART.md](./QUICKSTART.md)
2. ✅ 安裝 Blender 或 Isaac Sim
3. ✅ 執行 `blender_generate_1F.py` 建立第一個模型
4. ✅ 成功匯出 USD 並在 Isaac Sim 中檢視

### 本週目標
- [ ] 完成 1F 模型的完整流程測試
- [ ] 調整材質和照明達到滿意效果
- [ ] 開始建立 2F 托嬰中心腳本

### 本月目標
- [ ] 完成所有 5 層樓的建模
- [ ] 組裝完整建築場景
- [ ] 產出初步渲染圖

---

**讓我們開始建造這個跨齡共融的社福樞紐吧！🏗️✨**

---

*專案由 Claude 協助規劃與開發*
*最後更新: 2025-11-23*
