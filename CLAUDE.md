# SPIKE Prime Hub NuttX プロジェクト

SPIKE Prime Hub 上で NuttX RTOS を動作させる環境を構築するプロジェクト。

## Git / PR ワークフロー

タスク完了時、指示があればPR作成・更新まで一気通貫で実行する。

- **ブランチ**: `feat/`, `docs/`, `style/`, `fix/`, `build/`, `refactor/`, `chore/` prefix。ベースは常に `main`
- **コミット**: conventional commits 形式 `type: short description`

### PR作成

```bash
gh pr create --title "type: short description" --body "$(cat <<'EOF'
## Summary
- 変更点を箇条書き

## Test plan
- [x] テスト項目

🤖 Generated with [Claude Code](https://claude.com/claude-code)
EOF
)"
```

### PR更新

追加コミット & push 後、**必ずPRにコメントを残す**:

```
## type: short description (commit-hash)

変更内容の説明。
```

### PRマージ

```bash
gh pr merge <PR番号> --merge --delete-branch
git remote prune origin
```

## ドキュメント

`docs/ja/` と `docs/en/` に同名 `.md` を作成（日英必須）。

### カテゴリ

- `setup/` — 初期設定、環境構築
- `development/` — ビルド・実行ガイド
- `bringup/` — ブリングアップ調査・計画
- `drivers/` — デバイスドライバ設計・実装計画

### ソースコード参照

GitHub パーマリンク（コミットハッシュ + 行番号）を使用。ソース変更時はリンクを更新する。

## apps ディレクトリ

TBD

## ビルド環境

### pybricks (環境調査用)

pybricks-micropython v3.6.1 を git submodule として `./pybricks` に配置。Docker コンテナでビルド。

```bash
# フルビルド（submodule init → docker image build → firmware build）
make -f scripts/pybricks.mk

# クリーンビルド
make -f scripts/pybricks.mk clean

# 完全クリーン（docker image 削除 + submodule deinit）
make -f scripts/pybricks.mk distclean
```

### NuttX

NuttX 12.12.0 (owhinata fork) を git submodule として `./nuttx` と `./nuttx-apps` に配置。Docker コンテナでビルド。

```bash
# フルビルド（submodule init → docker image build → configure → make）
make -f scripts/nuttx.mk

# ボード指定ビルド
make -f scripts/nuttx.mk BOARD=stm32f4discovery BOARD_CONFIG=nsh

# Kconfig メニュー
make -f scripts/nuttx.mk menuconfig

# defconfig 保存
make -f scripts/nuttx.mk savedefconfig

# クリーンビルド
make -f scripts/nuttx.mk clean

# 完全クリーン（docker image 削除 + submodule deinit）
make -f scripts/nuttx.mk distclean
```

## デバイスアクセス

TBD

## 用語

- **Powered Up デバイス**: SPIKE Prime Hub の I/O ポートに接続するモーター・センサーの総称 (LEGO 公式ブランド名)
- **LPF2**: LEGO Power Functions 2。Powered Up デバイスが使用するコネクタ規格・プロトコル規格のコミュニティ通称
- **LUMP**: LEGO UART Messaging Protocol。Powered Up スマートデバイス (センサー/エンコーダ付きモーター) が Hub と通信する UART プロトコル

## SPIKE Prime Hub 仕様

- MCU: STM32F413 (ARM Cortex-M4)
- Flash: 1.5 MB
- RAM: 320 KB
