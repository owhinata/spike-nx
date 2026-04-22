# CC2564C Bluetooth (btstack + SPP)

SPIKE Prime Hub の **TI CC2564C** (BR/EDR + BLE デュアルモード Bluetooth コントローラ) を使い、Classic BT **SPP (Serial Port Profile over RFCOMM)** で PC (Linux / macOS) に IMU テレメトリをストリーミングするサブシステム。

Issue #47 で構築した NuttX 標準 BT スタックは LE 専用で RFCOMM / SDP を持たないため、Issue #52 で **btstack** (BlueKitchen) を取り込み NuttX 標準スタックを撤去した。現在の構成は 3 レイヤ:

- **ボード層** (`boards/spike-prime-hub/src/`): USART2 + DMA の電源・クロック管理と `/dev/ttyBT` chardev
- **btstack** (`libs/btstack/` submodule): HCI + L2CAP + RFCOMM + SDP + Classic BT + CC2564C chipset driver
- **アプリ層** (`apps/btsensor/`): btstack を NuttX user-mode で駆動する run loop / UART adapter、SPP サーバ、IMU サンプラ

## ハードウェア構成

| 信号 | ピン / 周辺機能 | 説明 |
|------|----------------|------|
| TX | PD5 (AF7) | USART2 TX |
| RX | PD6 (AF7) | USART2 RX |
| CTS | PD3 (AF7) | HW flow control 必須 |
| RTS | PD4 (AF7) | HW flow control 必須 |
| nSHUTD | PA2 (GPIO output) | CC2564C chip enable (Active HIGH、初期 LOW) |
| SLOWCLK | PC9 (AF3, TIM8 CH4) | 32.768 kHz 50% duty sleep clock (nSHUTD HIGH 前に安定化) |
| DMA RX | DMA1 Stream 7 Channel 6 | RM0430 Rev 9 Table 30 の F413 固有多重マッピング #2 (VERY_HIGH) |
| DMA TX | DMA1 Stream 6 Channel 4 | VERY_HIGH |
| NVIC 優先度 | 0xA0 (USART2, DMA1 S6, DMA1 S7) | Issue #50 予約枠 (LUMP=0x90 > BT=0xA0 > IMU=0xB0) |

ピン配線と DMA 割当は pybricks `lib/pbio/platform/prime_hub/platform.c` と一致。TIM8 CH4 の 32.768 kHz 生成は PSC=0, ARR=2929, CCR4=1465 (APB2 96 MHz / 2930 = 32.765 kHz, 誤差 -0.01%)。

## ソフトウェアレイヤ

```
┌────────────────────────────────────────────────────────────┐
│ apps/btsensor/ (user-mode, Issue #52 Step C-E)             │
│  btsensor_main.c   NSH builtin: `btsensor &`               │
│  btsensor_spp.c    L2CAP + RFCOMM + SDP + SSP Just-Works   │
│  imu_sampler.c     uORB accel/gyro → RFCOMM streaming       │
│  port/             btstack run loop + UART adapter          │
└────────────┬───────────────────────────────────────────────┘
             │ read/write/ioctl/poll on /dev/ttyBT
┌────────────▼───────────────────────────────────────────────┐
│ libs/btstack/                                              │
│  src/ hci.c l2cap.c btstack_run_loop_base.c …              │
│  src/classic/ rfcomm.c sdp_server.c spp_server.c …          │
│  chipset/cc256x/ btstack_chipset_cc256x.c (init script)    │
└────────────┬───────────────────────────────────────────────┘
             │ H4 over btstack_uart_t
┌────────────▼───────────────────────────────────────────────┐
│ boards/spike-prime-hub/src/ (kernel-mode)                  │
│  stm32_btuart.c          USART2 + DMA lower-half            │
│  stm32_btuart_chardev.c  /dev/ttyBT + poll()               │
│  stm32_bt_slowclk.c      TIM8 CH4 32.768 kHz PWM           │
│  stm32_bluetooth.c       nSHUTD toggle + slow clock + chardev│
│                          register — HCI は btstack が打つ  │
└────────────────────────────────────────────────────────────┘
```

### カーネル側 (`boards/spike-prime-hub/src/`)

- `stm32_btuart.c` — `struct btuart_lowerhalf_s` 実装 (USART2 + DMA1 S6/S7)。RX は 512 byte 循環 DMA + USART IDLE IRQ 駆動、TX は blocking DMA。API として `stm32_btuart_rx_available()` (非破壊ペック) を export。
- `stm32_btuart_chardev.c` — 上記 lower-half を `/dev/ttyBT` として POSIX chardev 化。`read`/`write`/`poll`/`ioctl` を実装。`ioctl(fd, BTUART_IOC_SETBAUD, baud)` で baud 変更。`poll()` 設定時は `rx_available > 0` で即 POLLIN 通知、POLLOUT は常時レディ。
- `stm32_bluetooth.c` — nSHUTD 制御と slow clock 起動、chardev 登録のみ。HCI reset / init script / baud 切替は btstack に委譲。
- `stm32_bt_slowclk.c` — TIM8 CH4 PWM (Issue #47 から変更なし)。

### ユーザ側 (`apps/btsensor/port/`)

btstack の公式 port/ ディレクトリに当たるレイヤを NuttX 用に書き起こした (`libs/btstack/port/stm32-wb55xx-nucleo-freertos/` や `platform/posix/` を参考):

- `btstack_run_loop_nuttx.c` — 単一スレッドの btstack run loop。データソースを `poll(2)` で待つ (POSIX 版を短くしたような実装)。ISR-drivenwake-up は `/dev/ttyBT` chardev の `poll_notify(POLLIN)` 経由。
- `btstack_uart_nuttx.c` — `btstack_uart_t` API を `/dev/ttyBT` に載せる。`receive_block`/`send_block` は data source の READ/WRITE フラグを立てるだけで、run loop の poll ディスパッチが実 I/O を発動する (btstack の POSIX UART と同じパターン)。
- `chipset/cc256x_init_script.c` — CC2564C v1.4 service pack (TI 公式 + eHCILL 無効化パッチ、pybricks baseline 由来)。`cc256x_init_script[]` / `cc256x_init_script_size` を export し、btstack の `btstack_chipset_cc256x.c` がこれを消費する。

### アプリ層 (`apps/btsensor/`)

- `btsensor_main.c` — NSH builtin `btsensor`。起動時に btstack を初期化し HCI_STATE_WORKING まで進め、SPP サーバ + IMU サンプラを登録して run loop に入る。永続的に動作するので `btsensor &` でバックグラウンド起動。
- `btsensor_spp.c` — L2CAP + RFCOMM + SDP のセットアップ、SPP SDP record 登録、SSP Just-Works 認証、RFCOMM チャネル開閉イベント処理。
- `imu_sampler.c` — uORB accel/gyro fd を btstack data source として登録し、16 サンプルを 1 フレームにまとめて RFCOMM で送信。

## Bring-up シーケンス (btstack 主導)

1. NuttX ブート中: `stm32_bluetooth_initialize()` が nSHUTD LOW → slow clock 起動 → USART2 lower-half instantiate → nSHUTD 50ms LOW / HIGH / 150ms 待機 → `/dev/ttyBT` を register。
2. NSH で `btsensor &` 起動後:
   - `btstack_run_loop_init` で NuttX run loop インスタンスを登録
   - `hci_init` + `hci_set_chipset(btstack_chipset_cc256x_instance())`
   - `spp_server_init` で L2CAP/RFCOMM/SDP/GAP を構成
   - `imu_sampler_init` で uORB fd を data source として登録
   - `hci_power_control(HCI_POWER_ON)` → btstack state machine が HCI Reset → Read_Local_Version → Read_Local_Supported_Commands → HCI_VS_Update_Baud_Rate (0xFF36) → chipset init script ストリーミング (~40 chunks、~200 ms) → Read_BD_ADDR → Write_Page_Scan_* → HCI_STATE_WORKING
3. `HCI working, BD_ADDR ...` が console に表示され、PC から discoverable に。

## SPP サービス仕様

- **Local name**: `SPIKE-BT-Sensor`
- **Class of Device**: `0x001F00` (Uncategorized)
- **Security**: SSP Just-Works (`SSP_IO_CAPABILITY_DISPLAY_YES_NO`), `LEVEL_2`
- **Service name**: `SPIKE IMU Stream`
- **RFCOMM channel**: 1
- **SDP UUID**: `0x1101` (SPP) + Profile Descriptor SPP v1.2

## RFCOMM ペイロード (IMU フレーム)

little-endian、1 フレーム = 12 byte header + 16 サンプル × 12 byte = 204 byte。

```c
struct spp_frame_hdr {
    uint16_t magic;          // 0xA55A
    uint16_t seq;            // フレーム単調連番
    uint32_t timestamp_us;   // Hub boot からの us
    uint16_t sample_rate;    // 情報用、現状 833 Hz 固定
    uint8_t  sample_count;   // 通常 16
    uint8_t  type;           // 0x01 = IMU
};

struct imu_sample {
    int16_t ax, ay, az;      // LSM6DS3TR-C accel 生 LSB (±8g, 0.244 mg/LSB)
    int16_t gx, gy, gz;      // LSM6DS3TR-C gyro  生 LSB (±2000 dps, 0.070 dps/LSB)
};
```

## 重要な設計決定

### btstack を選んだ理由

NuttX 標準 BT スタック (`CONFIG_WIRELESS_BLUETOOTH_HOST`) は BLE only で Classic BT の RFCOMM / SDP を持たないため、PC SPP 要求に対応できなかった。btstack は Classic + BLE 両対応、CC2564C chipset driver を標準装備、embedded / posix / freertos / zephyr の各 run loop 実装があり NuttX 向け port は `btstack_run_loop_posix.c` + `btstack_uart_posix.c` をベースに短時間で書けた。

### `/dev/ttyBT` chardev 方式

btstack は user-mode (BUILD_PROTECTED) で動作させる前提のため、HCI UART はカーネル → user へ POSIX `read`/`write`/`poll` で露出させる必要があった。既存の `struct btuart_lowerhalf_s` を chardev ラッパーで包み、btstack 側の `btstack_uart_t` はその fd を data source として扱う設計にした。

### `btuart_read` の `rxwork_pending` 再アーム (重要 fix)

`stm32_btuart.c` の `btuart_notify_rx` は rxcb 重複呼び出し抑止のため `rxwork_pending` ラッチを立てる。当初実装では **ring が空のまま呼ばれた read() でしかラッチを落とさなかった**。btstack の `hci_transport_h4` は状態マシン上「次に必要な正確な byte 数」だけを `read()` する (packet type=1 byte → event header=2 byte → event body=N byte) ため、ring をきっかり空にする read はあっても「空 ring で read」は発生しない。結果として初回バーストの後 `rxwork_pending` が永久に立ちっぱなしになり、IDLE ISR が発火しても rxcb がスキップされ、HCI 応答が 1 秒間隔 (poll timeout) でしか来ない症状が出た。

修正: `btuart_read` の末尾で producer==consumer を IRQ 排他下で再評価し、空なら `rxwork_pending = false` に戻す。実装は `boards/spike-prime-hub/src/stm32_btuart.c` の該当パッチを参照。

この fix により ISR 発火→user-mode poll wake のレイテンシは 10ms 以下まで落ち、CC2564C init script の全量 (~40 コマンド) が ~200 ms で流れる。

### CC2564C init script の eHCILL 無効化パッチ

pybricks baseline の v1.4 service pack は `HCI_VS_Sleep_Mode_Configurations` (opcode 0xFD0C) の eHCILL flag が `0x01` (enabled) になっている。eHCILL を有効にすると chip が idle 時に `GO_TO_SLEEP_IND (0x30)` を host に送り、host が `WAKE_UP_IND (0x31)` を返さない限りスリープに入り無応答になる。btstack の `hci_transport_h4` は `ENABLE_EHCILL` を定義しない限りこの ack を送らないので、シンプルに script 側の flag byte を `0x00` にパッチする形にした (Issue #47 commit 92817cb)。

### btstack run loop の構造

`btstack_run_loop_nuttx.c` は POSIX 版を NuttX 用に縮退させた単一スレッド実装:

- `execute()` はループで `poll(pfds, nfds, timeout_ms)` を呼ぶ
- `timeout_ms` は次の btstack タイマー期限 (`btstack_run_loop_base_get_time_until_timeout`) と `NUTTX_RUN_LOOP_MAX_WAIT_MS = 1000` の min
- POLLIN / POLLOUT で発火した data source の `process()` を呼ぶ
- タイマー処理 (`btstack_run_loop_base_process_timers`) と cross-thread callback (`execute_on_main_thread` 経由の `btstack_run_loop_base_execute_callbacks`) を毎ターン回す

`poll_data_sources_from_irq()` はフラグをセットするだけ (NuttX は fd-poll ベースなので実質的な wake-up は chardev 側の `poll_notify` に任せる)。

## Kconfig

```
# 有効化するもの
CONFIG_STM32_USART2=y
CONFIG_STM32_TIM8=y
CONFIG_SCHED_HPWORK=y          # (既存) uORB/btuart 用
CONFIG_SENSORS_LSM6DSL=y       # IMU uORB 公開
CONFIG_UORB=y
CONFIG_APP_BTSENSOR=y
CONFIG_APP_BTSENSOR_BATCH=16   # 1 RFCOMM frame に詰めるサンプル数
CONFIG_APP_BTSENSOR_RING_DEPTH=8
```

Issue #47 で使っていた `CONFIG_WIRELESS_BLUETOOTH_HOST` / `CONFIG_NET_BLUETOOTH` / `CONFIG_BLUETOOTH_UART_GENERIC` / `CONFIG_BTSAK` / `CONFIG_NETDEV_LATEINIT` は Issue #52 Step A で削除した。

## NSH 操作

```
nsh> ls /dev/ttyBT
/dev/ttyBT

nsh> dmesg | grep BT
BT: CC2564C powered, /dev/ttyBT ready

nsh> btsensor &
btsensor [5:100]
btsensor: bringing up btstack on /dev/ttyBT
btsensor: HCI working, BD_ADDR F8:2E:0C:A0:3E:64 — advertising as "SPIKE-BT-Sensor"
```

## PC 側受信

→ [PC での SPP 受信手順](../development/pc-receive-spp.md) を参照。

## 関連ドキュメント

- [PC での SPP 受信手順](../development/pc-receive-spp.md) — Linux / macOS ペアリングとストリーム読み出し
- [DMA / IRQ 割当台帳](../hardware/dma-irq.md) — DMA ストリーム・NVIC 優先度の全体設計
- [ピンマップ](../hardware/pin-mapping.md) — Bluetooth 節
- [テスト仕様書 H. Bluetooth](../testing/test-spec.md#h-bluetooth-test_bt_spppy) — pytest 自動/対話テスト

## 参照ソース

- btstack upstream: [bluekitchen/btstack](https://github.com/bluekitchen/btstack) (libs/btstack submodule、v1.8.1-6)
- btstack `platform/posix/btstack_run_loop_posix.c` — NuttX run loop の下敷き
- btstack `platform/posix/btstack_uart_posix.c` — UART wrapper の下敷き
- btstack `chipset/cc256x/btstack_chipset_cc256x.c` — init script 注入 + baud 切替ロジック
- btstack `example/spp_counter.c` — SPP server + SSP Just-Works の最小リファレンス
- pybricks `lib/pbio/drv/bluetooth/bluetooth_btstack_uart_block_stm32_hal.c` — STM32 HAL 上の参考 UART 実装
- TI: [CC256XC-BT-SP service pack](https://www.ti.com/tool/CC256XC-BT-SP) (init script 元ファイル)
- RM0430 Rev 9 §9.3.4 Figure 24 / Table 30 (DMA1 request mapping)
