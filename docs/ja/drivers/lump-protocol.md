# LUMP UART プロトコルエンジン

## 1. 概要

SPIKE Prime Hub の 6 つの I/O ポート (A–F) に挿された Powered Up スマートデバイス (LPF2 モーター/センサー) と通信する **LUMP (LEGO UART Messaging Protocol)** プロトコルエンジン。NuttX カーネル空間で動作し、後続の Motor (#44) / Sensor (#45) ドライバに `lump_*` API でモード情報とデータを供給する (Issue #43)。

参考実装は `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c` (1264 行)。pybricks の Contiki protothread モデルを NuttX の **kthread per port (6 thread)** に置き換えた。

## 2. アーキテクチャ

### 2.1 階層

```
[user]   apps/legoport/legoport_main.c (CLI)
   │
[user]   /dev/legoport[N] ioctl (LUMP_*)
   │
[kernel] stm32_legoport_chardev.c (chardev shim)
   │
[kernel] stm32_legoport_lump.c (engine, kthread × 6)
   │
[kernel] stm32_legoport_uart_hw.c (USART register wrapper)
   │
[hw]     UART4/5/7/8/9/10
```

### 2.2 スレッディング: kthread per port

- 6 個の kthread を `stm32_legoport_lump_register()` で boot 時に **pre-create** (priority `SCHED_PRIORITY_DEFAULT` = 100, stack 2048 B、計 12 KB)
- 起動直後に per-port `nxsem_t lump_wakeup` で sleep
- DCM (#42) のハンドオフ CB は HPWORK 192 コンテキストで実行され、`nxsem_post(&lump_wakeup[port])` だけで kthread を wake する → HPWORK 2 ms cadence は破壊しない
- HPWORK は採用しない (RX 待ちで 250 ms blocking が他ポートの DCM 進行を阻害するため)

### 2.3 NuttX serial driver はバイパス

- defconfig に `CONFIG_STM32_UART{4,5,7,8,9,10}` は **付けない**(自動的に `_SERIALDRIVER` が選ばれて `/dev/ttyS*` が登録されるため)
- `lump_uart_open()` から `modifyreg32(STM32_RCC_APB1ENR/APB2ENR, ...)` で RCC clock を手動 enable
- USART CR1/BRR/CR2/CR3 を直叩き、IRQ は `irq_attach + up_enable_irq` で per-port

### 2.4 NVIC 優先度: 0x90 同格 6 UART

`docs/{ja,en}/hardware/dma-irq.md:151` で 0x90 が LUMP UART 用に予約済み。`stm32_bringup.c` の `CONFIG_ARCH_IRQPRIO` ブロック内で UART4/5/7/8/9/10 を一律 0x90 に設定。

- pybricks は preempt=0 (最高) 同格、NuttX では BASEPRI 制約で 0x90 同格に圧縮 (相対順位は維持)
- 6 IRQ 同格運用前提のため ISR は **DR read + ring enqueue + ORE clear のみ**、`nxsem_post` は per-byte で叩かず `post_pending` flag で coalescing

## 3. 状態マシン

```
       lump_wakeup (DCM handoff CB)
              │
              ▼
   ┌──────[ IDLE ]──────┐
   │                    │
   │ open USART AF      │
   │ send CMD SPEED     │
   │  → fallback 2400   │
   │                    │
   ▼                    │
[ SYNCING ]             │
   │ wait CMD TYPE      │
   │ (10 retry on bad)  │
   ▼                    │
[ INFO ]                │
   │ NAME, RAW, PCT,    │
   │ SI, UNITS, MAPPING,│
   │ FORMAT × N modes   │
   │ → SYS_ACK from dev │
   │ send SYS_ACK       │
   │ wait 10 ms         │
   │ baud → 115200      │
   ▼                    │
[ DATA ] ◄── 100 ms NACK keepalive
   │                    │
   │ recv DATA frames   │
   │ → on_data callback │
   │ → ring buffer      │
   │ TX queue drain     │
   │  (CMD SELECT,      │
   │   CMD EXT_MODE,    │
   │   DATA writable)   │
   │                    │
   │ 6 missed keepalives│
   │   = 600 ms silent  │
   │   OR watchdog 2 s  │
   ▼                    │
[ ERR ] ── close →──────┘
   release_uart →
   register_uart_handoff →
   capped exp backoff (100ms → 1s → 5s → 30s)
```

## 4. ワイヤフォーマット (LUMP)

```
ヘッダバイト: [TT SSS CCC]
  TT (bit 7-6): メッセージ種別  SYS=00, CMD=01, INFO=10, DATA=11
  SSS (bit 5-3): ペイロードサイズ符号  0..5 = 1, 2, 4, 8, 16, 32 byte
  CCC (bit 2-0): SYS=システムコード / CMD=コマンド番号 / INFO/DATA=モード番号

SYS  : header のみ (1 byte)
CMD  : header + payload + checksum
INFO : header + info_type byte + payload + checksum
DATA : header + payload + checksum

checksum: 0xFF XOR all preceding bytes
```

主要コマンド/情報サブタイプは `pybricks/lib/lego/lego_uart.h` 準拠 (`LUMP_CMD_TYPE/MODES/SPEED/SELECT/WRITE/EXT_MODE/VERSION`、`LUMP_INFO_NAME/RAW/PCT/SI/UNITS/MAPPING/FORMAT`)。

## 5. 公開 API (`board_lump.h`)

```c
struct lump_device_info_s    /* type_id, num_modes, baud, modes[8] */
struct lump_mode_info_s      /* name, num_values, data_type, writable, raw/pct/si min/max, units */
struct lump_data_frame_s     /* mode, len, data[32] — DATA frame snapshot */
struct lump_status_full_s    /* state, type, mode, baud, rx/tx bytes, drops, backoff, stack high-water */

int lump_attach(int port, const struct lump_callbacks_s *cb);
int lump_detach(int port);
int lump_select_mode(int port, uint8_t mode);
int lump_send_data(int port, uint8_t mode, const uint8_t *buf, size_t len);
int lump_get_info(int port, struct lump_device_info_s *out);
int lump_get_status(int port, uint8_t *flags, uint32_t *rx, uint32_t *tx);
int lump_get_status_full(int port, struct lump_status_full_s *out);
```

`lump_attach` 時に既に SYNCED なら `on_sync` を同期発火 (lock 解放後)。CB context からの同一ポート再入は `-EDEADLK`。

### 5.1 コールバック発火タイミング (Issue #76)

| コールバック | 発火元 | 発火タイミング |
|---|---|---|
| `on_sync` | per-port kthread | `SYNCING -> DATA` への遷移直後 (毎セッション、re-sync 後も) — `info` は SYNCED 状態の snapshot を渡す |
| `on_sync` | 呼び出し元 thread | `lump_attach()` 時に engine が既に SYNCED なら同期発火 |
| `on_data` | per-port kthread | DATA frame を受信し parse 成功するごと、`current_mode` 更新後 |
| `on_error` | per-port kthread | DATA loop が抜けた直後 (sync 失敗 / keepalive miss / watchdog stall いずれも)、`release_uart` 後の clean state で `data == NULL && len == 0` を引数に発火。コンシューマは disconnect sentinel をここで publish できる |

すべて per-port kthread / API context で `cb_lock` を一旦解放してから fire するので、CB から `lump_get_info` / `lump_get_status` 等は安全に呼べる。同一ポート CB 内からの `lump_attach` / `lump_detach` / 別 CB 経路への再入は `-EDEADLK` で拒否される (`in_callback` flag)。

`on_error` は **接続中だけでなく sync 失敗時にも発火** する点に注意 — 一度も `on_sync` を受けていない状態でも disconnect 通知が届きうる。コンシューマ側は idempotent (同じ disconnect を二度受け取っても安全) に書く。

## 6. ioctl (`/dev/legoport[N]`)

| ioctl | arg | 動作 |
|---|---|---|
| `LEGOPORT_LUMP_GET_INFO` | `lump_device_info_s *` | SYNCED 時のみ完全 info、未同期は -EAGAIN |
| `LEGOPORT_LUMP_SELECT` | `uint8_t mode` | CMD SELECT を kthread に queue |
| `LEGOPORT_LUMP_SEND` | `legoport_lump_send_arg_s *` | DATA TX を queue (writable mode) |
| `LEGOPORT_LUMP_POLL_DATA` | `lump_data_frame_s *` | DATA frame ring から 1 frame pop、空は -EAGAIN |
| `LEGOPORT_LUMP_GET_STATUS_EX` | `lump_status_full_s *` | per-port full status snapshot |
| `LEGOPORT_LUMP_HW_DUMP` | (なし) | RCC/USART/NVIC レジスタを syslog 出力 (`CONFIG_LEGO_LUMP_DIAG`) |

## 7. CLI (`legoport lump <subcommand>`)

```
legoport lump status              - per-port engine state テーブル
legoport lump info <N>            - 完全 device_info dump
legoport lump set-mode <N> <m>    - CMD SELECT 要求
legoport lump send <N> <m> <hex>...  - DATA TX (writable mode、1..32 byte hex pair)
legoport lump watch <N> <ms>      - DATA frame を ms ミリ秒間 dump (10 ms poll)
legoport lump-hw dump             - RCC/USART/NVIC dump (DIAG ビルドのみ)
```

## 8. ハンドオフ契約

DCM (#42) との連携:

1. 起動時: 6 ポート分の `lump_handoff_cb` を `stm32_legoport_register_uart_handoff(port, cb, &g_lump[port])` で登録
2. DCM が `UNKNOWN_UART` 確定 → CB が呼ばれる (HPWORK ctx) → CB は pins を保存して `nxsem_post(&wakeup)`
3. kthread wake → SYNC → DATA loop
4. disconnect / error 時: kthread は **`close → release_uart → register_uart_handoff → backoff sleep`** を順に実行 (cooldown は release/register の前後ではなく後)
5. release_uart は CB を NULL クリアするので、必ず即時に register_uart_handoff() で再登録する

## 9. チューニングノブ

| 定数 | 値 | 説明 |
|---|---|---|
| `LUMP_BAUD_INITIAL` | 115200 | 初期 SPEED probe baud |
| `LUMP_BAUD_FALLBACK` | 2400 | EV3 互換 fallback |
| `LUMP_BAUD_MAX` | 460800 | デバイス指定の上限 |
| `LUMP_IO_TIMEOUT_MS` | 250 | SYNC/INFO 1 byte read 上限 |
| `LUMP_DATA_RECV_SLICE_MS` | 20 | DATA loop での 1 frame 受信 slice |
| `LUMP_KEEPALIVE_MS` | 100 | `SYS_NACK` 送信周期 |
| `LUMP_DATA_MISS_LIMIT` | 6 | キープアライブ無 data 連続上限 (= 600 ms で disconnect) |
| `LUMP_WATCHDOG_MS` | 2000 | DATA loop 1 iteration の上限 |
| `LUMP_DATA_QUEUE` | 16 | per-port DATA frame ring 深度 |
| `LUMP_UART_RXRING_SIZE` | 256 | per-port RX ring (ISR 側) |
| `LUMP_UART_TX_BYTE_TIMEOUT_MS` | 10 | TX 1 byte poll-TXE 上限 |
| Backoff schedule | `100/1000/5000/30000 ms` | capped exp、5 連続失敗で `LUMP_FAULT_BACKOFF` |

## 10. ライセンス

`stm32_legoport_lump.c` は pybricks `legodev_pup_uart.c` からの port。SPDX `MIT` (pybricks の dual licensing から MIT 側を選択)、Copyright 表示 `(c) 2018-2023 The Pybricks Authors`。

## 11. 参照

- 設計詳細: [port-detection.md](port-detection.md) §4 (LUMP プロトコル概要 + DCM ハンドオフ)
- リソース台帳: `docs/{ja,en}/hardware/dma-irq.md`
- pybricks 元実装: `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c`、`pybricks/lib/lego/lego_uart.h`
