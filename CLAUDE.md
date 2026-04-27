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
6. **コミット** — 動作確認後にコミット

動作確認前にコミットしない。ドキュメント更新を忘れない。

## Git / Issue ワークフロー

PR は作成しない。Issue 駆動で feature/fix ブランチを切り、ローカル `main` に ff-merge → push → Issue へ対応コメント → Issue クローズ、の流れ。

- **ブランチ**: `feat/`, `docs/`, `style/`, `fix/`, `build/`, `refactor/`, `chore/` prefix。ベースは常に `main`
- **コミット**: conventional commits 形式 `type: short description`。Issue 修正時は `type: #N short description` で Issue 番号を含める (GitHub のリンク生成 + マージ後の自動クローズ判定のため)

### 1. Issue 作成

着手前にまず Issue を立てる。バグ報告 / 機能追加 / リファクタ いずれも同様。

```bash
gh issue create --repo owhinata/spike-nx --title "short description" --body "$(cat <<'EOF'
## Summary
- 症状・問題・やりたいことの説明

## Reproduction (バグの場合)
- 再現手順

## Environment
- Board: SPIKE Prime Hub (STM32F413)
- Reproduced at: <commit-hash>
- 関連する CONFIG 設定

## Notes
- 調査メモ・仮説・設計案
EOF
)"
```

### 2. ブランチを切って実装 → 動作確認 → コミット

```bash
git checkout -b feat/<N>-short-description    # <N> は Issue 番号
# ... 実装 → ビルド → フラッシュ → 動作確認 ...
git commit -m "type: #<N> short description"
```

### 3. ローカル main に ff-merge

```bash
git checkout main
git merge --ff-only feat/<N>-short-description
```

ff-only で必ず early return する。merge コミットは作らない。

### 4. push

```bash
git push origin main
```

push 時、コミットメッセージに `fix: #<N> ...` / `closes #<N>` 等の GitHub オートクローズキーワードが含まれていれば、対応する Issue は自動的に CLOSED になる。

### 5. Issue へ対応コメント

push 後、`gh issue comment` で対応内容のサマリを残す。コミットレンジ (`<base>..<head>`) を必ず含める。

```bash
gh issue comment <N> --repo owhinata/spike-nx --body "$(cat <<'EOF'
## 対応完了

`feat/<N>-...` を `main` に ff-merge して push (`<base>..<head>`)。

### 修正内容
- 変更点を箇条書き

### 動作確認
- 実機で確認した挙動

### 補足 (任意)
- 関連 Issue / 後続フォローアップ
EOF
)"
```

### 6. Issue クローズ + ブランチ削除

オートクローズで既に CLOSED になっていなければ手動で閉じる。マージ済みブランチは削除する。

```bash
gh issue close <N> --repo owhinata/spike-nx          # 必要なら
git branch -d feat/<N>-short-description
```

### 注意

- **PR は作らない**。`gh pr create` / `gh pr merge` は使わない
- `main` への直 push なので `--ff-only` を厳守、force push は禁止
- 動作確認していないコミットを `main` に push しない (CLAUDE.md の hook が `未実施` 系の文言を含むコミットメッセージをブロックする)
- Issue を立てずにコミットしない (`#<N>` 参照が無いコミットは追跡できない)

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
# usbnsh (既定) は BUILD_PROTECTED なので kernel + user の 2 段書き込み
dfu-util -d 0694:0008 -a 0 -s 0x08008000 -D nuttx/nuttx.bin
dfu-util -d 0694:0008 -a 0 -s 0x08080000:leave -D nuttx/nuttx_user.bin
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
- テスト依存: `.venv/bin/pip install -r requirements.txt`

### `tests/conftest.py` を使ったアドホックデバッグ

実機で NSH コマンドを叩きながら挙動を確認したいときは、`tests/conftest.py` の `NuttxSerial` ヘルパーを Python から直接呼ぶのが一番速い。pytest を介さないので出力がそのままチャット / ターミナルに流れる。

```python
import sys, time
sys.path.insert(0, 'tests')
from conftest import NuttxSerial, PROMPT

s = NuttxSerial('/dev/tty.usbmodem01', 'tests/logs')
try:
    s.proc.sendline('')           # 最初のプロンプト待ち
    s.proc.expect(PROMPT, timeout=5)

    out = s.sendCommand('ls /dev')
    print(out)

    # dmesg は一度読み取ると drain されるので、起動ログを見たいときは
    # reboot 直後に取得する
    out = s.sendCommand('dmesg')
    print(out)
finally:
    s.close()
```

**`sendCommand()` を必ず使う** — 生の `s.proc.send('cmd\r\n') + s.proc.expect(PROMPT)` はプロンプトマッチが壊れて `before` が空文字列になる。`sendCommand` はコマンドエコーの先頭単語 → プロンプトの順に待つので、出力の回収を確実にできる。

よくあるミスと対策:
- **`dmesg` が空に見える**: RAMLOG は read で drain されるので、直前の接続で既に読まれている可能性。`reboot` した直後に読むか、実機を電源再投入する。
- **`free` 出力の ValueError**: `conftest.getFree()` が現在の NuttX `free` 出力フォーマットと合っておらず、`check_memory_leak` フィクスチャがテスト失敗後にパースエラーを出すことがある。本体テストがすでに失敗しているときに後から出る二次エラーなので無視して良い。
- **NSH のクォート挙動**: `cmd "a b c"` の引用符は NSH が argv に分割する段階で外される場合がある。`apps/*_main.c` 側で `argv[2..argc-1]` を自前で join する必要がある。
- **シリアル接続後の最初の 1 行が空振る**: USB CDC の取りこぼしが起きやすいので `sendline('')` を 1〜2 回送ってから `expect(PROMPT)` するとプロンプトに同期できる。

## Upstream リポジトリへの操作制限

以下の upstream リポジトリに対して `gh` コマンドによる書き込み操作（create, comment, close, merge, edit, delete 等）を行ってはならない。読み取り操作（view, list）は許可する。

- `apache/nuttx`
- `apache/nuttx-apps`
- `pybricks/pybricks-micropython`

`gh` コマンドは常に `--repo owhinata/spike-nx` を明示すること。`.claude/settings.json` の PreToolUse フックで書き込み操作は自動ブロックされる。

## 注意事項

- NuttX の Kconfig にペリフェラルの定義があっても、実際の MCU にそのハードウェアが存在するとは限らない。Kconfig はチップファミリ単位（例: STM32F4XXX）の粗い分類で、個別チップの差分を反映していないことがある。ペリフェラルの有無は必ずリファレンスマニュアル（データシート）で確認すること。
  - 例: STM32F413 は STM32F4XXX に分類されるため `CONFIG_STM32_BKPSRAM` が有効化可能だが、実際には BKPSRAM を持たない（RM0430 参照）。有効にするとコンパイルは通るが実行時に HardFault になる。

## SPIKE Prime Hub 仕様

- MCU: STM32F413 (ARM Cortex-M4)
- Flash: 1.5 MB
- RAM: 320 KB
