# SPIKE Prime Hub NuttX プロジェクト

SPIKE Prime Hub 上で NuttX RTOS を動作させる環境を構築するプロジェクト。

## 開発ワークフロー

### コード修正サイクル

以下のサイクルを小さく繰り返す:

1. **コード修正** — 機能実装 or バグ修正
2. **ビルド** — `make` (差分ビルド) or `make nuttx-distclean && make` (フルビルド)
3. **フラッシュ** — `dfu-util` で実機に書き込み
4. **動作確認** — ユーザーが実機で手動テスト
5. **ドキュメント更新** — 変更に対応するドキュメントを日英で更新
6. **コミット & PR 更新** — push 後 PR にコメントを残す

動作確認前にコミットしない。ドキュメント更新を忘れない。

## Git / PR ワークフロー

タスク完了時、指示があればPR作成・更新まで一気通貫で実行する。

- **ブランチ**: `feat/`, `docs/`, `style/`, `fix/`, `build/`, `refactor/`, `chore/` prefix。ベースは常に `main`
- **コミット**: conventional commits 形式 `type: short description`。Issue 修正時は `type: #N short description` で Issue 番号を含める（GitHub リンク生成のため）

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

### Issue 作成

バグや課題を発見したら Issue を作成する。Environment に reproduced at (再現確認リビジョン) を必ず含める。

```bash
gh issue create --title "short description" --body "$(cat <<'EOF'
## Summary
- 症状・問題の説明

## Reproduction
- 再現手順

## Environment
- Board: SPIKE Prime Hub (STM32F413)
- Reproduced at: <commit-hash>
- 関連する CONFIG 設定

## Notes
- 調査メモ・仮説
EOF
)"
```

## ドキュメント

`docs/ja/` と `docs/en/` に同名 `.md` を作成（日英必須）。mkdocs (Material テーマ + i18n) で GitHub Pages に publish。

### カテゴリ

- `hardware/` — ハードウェア仕様・ピンマップ・ペリフェラル
- `drivers/` — ドライバアーキテクチャ・実装計画
- `development/` — ビルド・デバッグ・開発ワークフロー
- `nuttx/` — NuttX カスタマイズ（F413 チップサポート等）

### ローカルプレビュー

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
mkdocs serve        # http://localhost:8000
mkdocs build --strict  # ビルド検証
```

### デプロイ

main ブランチへの push 時に GitHub Actions で自動デプロイ（`docs/**`, `mkdocs.yml`, `requirements.txt` の変更時のみ）。

### ソースコード参照

GitHub パーマリンク（コミットハッシュ + 行番号）を使用。ソース変更時はリンクを更新する。

## apps ディレクトリ

`apps/` にカスタムアプリを配置。NuttX ビルドシステムの External Application として統合される。

### 構成

各アプリは `apps/<name>/` に以下のファイルを配置:

- `Kconfig` — `CONFIG_APP_<NAME>` 定義
- `Makefile` — `PROGNAME`, `MODULE`, `MAINSRC` 設定
- `Make.defs` — `CONFIGURED_APPS += $(APPDIR)/external/<name>`
- `<name>_main.c` — エントリポイント `<name>_main()`

### アプリ追加手順

1. `apps/<name>/` ディレクトリを作成し上記ファイルを配置
2. `apps/Kconfig` に `source` 行を追加
3. defconfig に `CONFIG_APP_<NAME>=y` を追加

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

NuttX 12.13.0 (owhinata fork) を git submodule として `./nuttx` と `./nuttx-apps` に配置。Docker コンテナでビルド。

```bash
# フルビルド（SPIKE Prime Hub, usbnsh）
make

# Kconfig メニュー
make nuttx-menuconfig

# defconfig 保存
make nuttx-savedefconfig

# クリーンビルド（.config は残る）
make nuttx-clean

# .config も含めて完全クリーン（BOARD 切り替え時はこちらを使う）
make nuttx-distclean

# 完全クリーン（docker image 削除 + submodule deinit）
make distclean
```

アプリの ELF ビルド定義は `apps/<app>/elf.mk` に記載。ELF ローダー有効化は `feat/elf-loader` ブランチを参照。

## デバイスアクセス

### SPIKE Prime Hub (DFU)

```bash
# DFU モード: USB 抜く → Bluetooth ボタン押したまま USB 接続 → 5秒待って離す
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```

### シリアル接続

```bash
picocom /dev/tty.usbmodem01
```

## 用語

- **Powered Up デバイス**: SPIKE Prime Hub の I/O ポートに接続するモーター・センサーの総称 (LEGO 公式ブランド名)
- **LPF2**: LEGO Power Functions 2。Powered Up デバイスが使用するコネクタ規格・プロトコル規格のコミュニティ通称
- **LUMP**: LEGO UART Messaging Protocol。Powered Up スマートデバイス (センサー/エンコーダ付きモーター) が Hub と通信する UART プロトコル

## テスト

`tests/` に pexpect + pyserial + pytest ベースの自動テストを配置。シリアル経由で NSH コマンドを実行し出力を検証する。

- 機能を実装したら対応するテストを `tests/` に追加する
- 既存テストを実行して既存機能が壊れていないことを確認する
- テスト実行: `.venv/bin/pytest tests/ -m "not slow" -D /dev/tty.usbmodem01`
- 自動テストのみ: `-m "not slow and not interactive"`
- OSテスト（ostest, coremark）はカーネル CONFIG 変更時のみ実行
- テスト依存: `.venv/bin/pip install -r tests/requirements.txt`

## 注意事項

- NuttX の Kconfig にペリフェラルの定義があっても、実際の MCU にそのハードウェアが存在するとは限らない。Kconfig はチップファミリ単位（例: STM32F4XXX）の粗い分類で、個別チップの差分を反映していないことがある。ペリフェラルの有無は必ずリファレンスマニュアル（データシート）で確認すること。
  - 例: STM32F413 は STM32F4XXX に分類されるため `CONFIG_STM32_BKPSRAM` が有効化可能だが、実際には BKPSRAM を持たない（RM0430 参照）。有効にするとコンパイルは通るが実行時に HardFault になる。

## SPIKE Prime Hub 仕様

- MCU: STM32F413 (ARM Cortex-M4)
- Flash: 1.5 MB
- RAM: 320 KB
