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
│ apps/btsensor/ (user-mode)                                 │
│  btsensor_main.c   NSH builtin + daemon + BT state machine │
│  btsensor_spp.c    L2CAP + RFCOMM + SDP + SSP Just-Works   │
│  btsensor_tx.c     RFCOMM send arbiter (resp + frame ring) │
│  btsensor_button.c BT 制御ボタンイベント (open /dev/btbutton)│
│  btsensor_led.c    BT LED ヘルパ (open /dev/rgbled0)        │
│  imu_sampler.c     uORB sensor_imu → RFCOMM streaming       │
│  port/             btstack run loop (self-pipe wake)        │
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
- `stm32_btuart_chardev.c` — 上記 lower-half を `/dev/ttyBT` として POSIX chardev 化。`read`/`write`/`poll`/`ioctl` を実装。
  - `ioctl(fd, BTUART_IOC_SETBAUD, baud)` で baud 変更
  - `ioctl(fd, BTUART_IOC_CHIPRESET, 0)` で CC2564C を nSHUTD パルスでリセット (Issue #56 follow-up: btstack `hci_power_off` 後はチップが post-init-script 状態に残り、次の `hci_init` で WORKING に到達しないため、btsensor デーモン起動毎にこの ioctl で chip を cold-boot 相当に戻す)
  - `poll()` 設定時は `rx_available > 0` で即 POLLIN 通知、POLLOUT は常時レディ
- `stm32_bluetooth.c` — nSHUTD 制御と slow clock 起動、chardev 登録、`stm32_bluetooth_chip_reset()` (上記 ioctl のバックエンド)。HCI reset / init script / baud 切替は btstack に委譲。
- `stm32_bt_slowclk.c` — TIM8 CH4 PWM (Issue #47 から変更なし)。

### ユーザ側 (`apps/btsensor/port/`)

btstack の公式 port/ ディレクトリに当たるレイヤを NuttX 用に書き起こした (`libs/btstack/port/stm32-wb55xx-nucleo-freertos/` や `platform/posix/` を参考):

- `btstack_run_loop_nuttx.c` — 単一スレッドの btstack run loop。データソースを `poll(2)` で待つ (POSIX 版を短くしたような実装)。ISR-drivenwake-up は `/dev/ttyBT` chardev の `poll_notify(POLLIN)` 経由。
- `btstack_uart_nuttx.c` — `btstack_uart_t` API を `/dev/ttyBT` に載せる。`receive_block`/`send_block` は data source の READ/WRITE フラグを立てるだけで、run loop の poll ディスパッチが実 I/O を発動する (btstack の POSIX UART と同じパターン)。
- `chipset/cc256x_init_script.c` — CC2564C v1.4 service pack (TI 公式 + eHCILL 無効化パッチ、pybricks baseline 由来)。`cc256x_init_script[]` / `cc256x_init_script_size` を export し、btstack の `btstack_chipset_cc256x.c` がこれを消費する。

### アプリ層 (`apps/btsensor/`)

- `btsensor_main.c` — NSH builtin `btsensor start [batch]` / `stop` / `status`。Issue #56 Commit B でフォアグラウンドの `btsensor &` から daemon 化。`task_create` で `btsensor_d` task を生成、btstack run loop を回す。`stop` は teardown FSM (GAP off → RFCOMM disconnect → sampler/tx deinit → HCI off → run loop exit) を main thread 上で進める。各 pending state には 3 秒の btstack timer watchdog が付く。Commit C で **BT 状態機械** (OFF / ADVERTISING / FAIL_BLINK / PAIRED) と短押し/長押しハンドラを追加し、ボタン (btsensor_button) と LED (btsensor_led) を駆動する。
- `btsensor_button.c` — `/dev/btbutton` (kernel-side ADC ladder polling) を btstack data source として登録し、`CONFIG_APP_BTSENSOR_BTN_LONG_PRESS_MS` (default 1500ms) で短押し/長押しを判別。コールバックは `btstack_run_loop_execute_on_main_thread()` 経由で main thread に集約する。
- `btsensor_led.c` — `/dev/rgbled0` の BT_B/G/R (CH18-20) を ioctl `RGBLEDIOC_SETDUTY` で制御。off / solid_blue / blink_blue (`CONFIG_APP_BTSENSOR_LED_BLINK_PERIOD_MS`) / fail_blink (`CONFIG_APP_BTSENSOR_LED_FAIL_BLINKS`) の 4 モード。点滅は btstack timer 駆動。
- `btsensor_spp.c` — L2CAP + RFCOMM + SDP のセットアップ、SPP SDP record 登録、SSP Just-Works 認証、RFCOMM チャネル開閉イベント処理。**起動時は `gap_discoverable_control(0) / gap_connectable_control(0)`**（Issue #56 Commit C で BT ボタンが advertising を有効化する）。
- `btsensor_tx.c` — 単一 RFCOMM 送信 arbiter（Commit B 新規）。応答用キュー (Commit D で使用、優先送信) と IMU フレーム用リングバッファを持ち、`RFCOMM_EVENT_CAN_SEND_NOW` で順次 drain する。`request_can_send_now` 要求は arbiter 一元化されているので、imu_sampler / コマンド応答が衝突しない。
- `btsensor_cmd.c` — Commit D 新規。`RFCOMM_DATA_PACKET` で受信した PC からのバイト列を 64B 行バッファに溜め、`'\n'` で `process_line()` を呼んで `strtok_r` ディスパッチ。応答 (`OK\n` / `ERR <reason>\n`) は `btsensor_tx_enqueue_response()` で送る。コマンドは `IMU ON/OFF` と `SET ODR/BATCH/ACCEL_FSR/GYRO_FSR <val>` のみ。
- `imu_sampler.c` — uORB `/dev/uorb/sensor_imu0` の fd を btstack data source として登録し、各 `struct sensor_imu` (accel/gyro 生 int16 + ISR 取得 timestamp) をワイヤフォーマットのサンプルスロットへコピー、Kconfig 既定 8 サンプル (1〜80 範囲) を 1 フレームにまとめて `btsensor_tx_try_enqueue_frame()` 経由で送信。Commit D で `IMU ON/OFF` トグルとキャッシュ付き SET API (`set_odr_hz` / `set_batch` / `set_accel_fsr` / `set_gyro_fsr` — IMU OFF 中のみ動作) を追加。

## NSH コマンド

```
nsh> btsensor start                       # daemon 起動 (BT adv / IMU 共に off)
nsh> btsensor status                      # running / pid / bt / imu / config / rfcomm / stats
nsh> btsensor stop                        # teardown FSM (HCI off まで非同期)
nsh> btsensor bt    <on|off>              # BT 可視性 (BT ボタン短押し/長押しと等価)
nsh> btsensor imu   <on|off>              # IMU sampling on/off (BT 状態と独立)
nsh> btsensor dump  [ms]                  # /dev/uorb/sensor_imu0 を直接読んで raw int16 + timestamp ダンプ
nsh> btsensor set   odr        <hz>       # ODR 変更 (IMU off 中のみ)
nsh> btsensor set   batch      <n>        # フレームあたりサンプル数 (IMU off 中のみ)
nsh> btsensor set   accel_fsr  <g>        # accel FSR (IMU off 中のみ)
nsh> btsensor set   gyro_fsr   <dps>      # gyro FSR (IMU off 中のみ)
```

batch サイズは `btsensor set batch <n>` (IMU off 中) で動的に変更する。multiple-start は `pid > 0` チェックで弾く。`stop` / `bt` / `imu` / `set` は `btstack_run_loop_execute_on_main_thread()` 経由で main thread 上の FSM / sampler を呼び、`sem_timedwait()` (3 秒 timeout) で同期結果を NSH に返す。`dump` のみ NSH context から直接 uORB topic を読むので daemon が無くても動く (kernel 側 driver は first subscriber で auto-activate)。

`status` 出力例:

```
running:    yes
pid:        9
bt:         off               # off / advertising / fail_blink / paired
imu:        on                # on / off
config:     odr=416Hz batch=8 accel_fsr=4g gyro_fsr=2000dps
rfcomm cid: 0
frames:     sent=0 dropped=194  # rfcomm cid==0 中はすべて drop
```

`dump` 出力例 (1 行 = 1 サンプル):

```
# ts_us ax ay az gx gy gz (raw int16, Hub body frame)
43933310 -76 32 -8220 -5342 -5929 -5789
43935640 -57 20 -8214 -5342 -5927 -5789
...
# 14 sample(s) over 200 ms
```

PC 接続なしでセンサ値を確認したいときは `btsensor dump <ms>` が便利。RFCOMM 経由の `IMU ON/OFF` / `SET *` (Commit D) と同じ動作が NSH からも完結する。

## BT 状態機械

```
   OFF ──short/long press──▶ ADVERTISING ──pairing OK──▶ PAIRED
                                  │                          │
                                  │  pairing FAIL            │  long press
                                  ▼                          ▼
                             FAIL_BLINK ─auto──▶ OFF         OFF
                                                              ▲
                                  PAIRED ─link drop──▶ CONNECTABLE
                                                       │  ▲
                                  short/long press     │  │  RFCOMM open
                                  / `bt on`            │  │  (再接続: 保存済
                                                       │  │   link key)
                                                       ▼  │
                                                  ADVERTISING ────▶ PAIRED
```

- 状態は `OFF` / `ADVERTISING` / `CONNECTABLE` / `FAIL_BLINK` / `PAIRED` の 5 つ。`status` コマンドで `bt: <state>` として表示
- 短押し: OFF → ADVERTISING のみ作用 (それ以外は no-op)
- 長押し: 全状態で意味あり — OFF/FAIL_BLINK → ADVERTISING、ADVERTISING/CONNECTABLE → OFF、PAIRED → RFCOMM 切断 + OFF
- `btsensor bt on` を CONNECTABLE 状態で叩くと ADVERTISING に昇格 (新規 PC の inquiry にも応答するようになる)。それ以外の状態は従来通り
- LED: OFF=消灯、ADVERTISING / CONNECTABLE=青 1Hz 点滅 (見た目は同一、区別は `btsensor status`)、PAIRED=青常時点灯、FAIL_BLINK=`CONFIG_APP_BTSENSOR_LED_FAIL_BLINKS` 回 (~150ms × 2N) の青パルス
- Pairing 完了は `HCI_EVENT_SIMPLE_PAIRING_COMPLETE`: status==0 で PAIRED (LED 即時点灯、Issue #56 仕様 "ペアリング成功で BT LED 点灯" に合わせる)、status≠0 で FAIL_BLINK。後続の `RFCOMM_EVENT_CHANNEL_OPENED` は PAIRED のまま (LED 変化なし)。リンク切断時は CONNECTABLE に遷移 (connectable=1, discoverable=0、LED は 1Hz 点滅再開) し、ペアリング済 PC は BD_ADDR で silent に再接続できるが新規 PC からは inquiry に出ない。Issue #68
- 状態遷移は必ず BTstack main thread 上で実行 (worker / shell からは `execute_on_main_thread()` 経由)

## PC コマンドプロトコル (Commit D)

PC が RFCOMM チャネル open 後に送る ASCII 1 行 / コマンド (`\n` 終端、`\r` 無視)。応答は `OK\n` か `ERR <reason>\n`。

| コマンド | 動作 | 制約 |
|---|---|---|
| `IMU ON\n` | サンプリング開始 (driver fd を open) | — |
| `IMU OFF\n` | サンプリング停止 (driver fd を close = 自動 deactivate) | — |
| `SET ODR <hz>\n` | ODR を変更 (13/26/52/104/208/416/833/1660/3330/6660 Hz) | IMU OFF 時のみ |
| `SET BATCH <n>\n` | フレームあたりサンプル数 (1〜80) | IMU OFF 時のみ |
| `SET ACCEL_FSR <g>\n` | 加速度 FSR (2/4/8/16) | IMU OFF 時のみ |
| `SET GYRO_FSR <dps>\n` | ジャイロ FSR (125/250/500/1000/2000) | IMU OFF 時のみ |

応答パターン:
- 成功: `OK\n`
- IMU ON 中の SET: `ERR busy\n`
- 不正な値 / トークン: `ERR invalid <token>\n`
- 行が 64B 超: `ERR overflow\n`
- 未知のコマンド: `ERR unknown <cmd>\n`

実装メモ:
- driver fd は IMU OFF 時 close されているため、SET は **transient な O_WRONLY fd** を一時的に open して ioctl を発行する。`O_WRONLY` はセンサ upper-half の自動 activate (O_RDOK でのみ発火) を回避するので、SET は driver の `if (active) -EBUSY` チェックを通過できる
- 設定値はアプリ側 `imu_sampler.c` のローカル変数 (`g_odr_hz` / `g_accel_fsr_g` / `g_gyro_fsr_dps` / `g_batch`) に保持。**ioctl 失敗時は前値にロールバック**する
- driver 側 (`lsm6dsl_uorb.c`) は `cfg_odr` フィールドで「ユーザ設定の ODR」を保持し、`activate(true)` 時に `set_odr(cfg_odr)` で適用する。`SET ODR` は activate サイクルを跨いで保持される

PC 側コードは `docs/{ja,en}/development/pc-receive-spp.md` で例示する予定 (Commit E)。

## EXTI0 / NVIC priority

`stm32_bringup.c` step 9 で `up_prioritize_irq(STM32_IRQ_EXTI0, NVIC_SYSH_PRIORITY_DEFAULT + 6 * NVIC_SYSH_PRIORITY_STEP) = 0xE0` を設定。BUTTON_USER (PA0 EXTI0) を ε 設計の最下位ペリフェラル帯 (ADC / TLC5955 と同 0xE0) に揃える。詳細は `docs/{ja,en}/hardware/dma-irq.md` の NVIC 表を参照。

## Self-pipe wake (run loop)

`apps/btsensor/port/btstack_run_loop_nuttx.c` は cross-thread `execute_on_main_thread()` を **self-pipe wake fd** で起こす:
- `pipe(p)` を init で作成し、O_NONBLOCK 化
- `g_wake_lock` (pthread_mutex) で `g_wake_wfd` への write を保護
- main thread の poll() に rfd を data source として加える
- callback enqueue 時に "x" を 1 byte 書き込む → POLLIN で run loop 起床

これにより cmd_stop / button IRQ / pairing event の cross-thread post が ms オーダーで反映される (従来は最大 1000 ms の poll timeout 待ち)。

## 既知のフォローアップ

- **BT 制御ボタンの物理検出**: `boards/spike-prime-hub/src/stm32_btbutton.c` は PA1 ADC ladder を polling して `/dev/btbutton` 経由でアプリへ通知する設計だが、実機ではボタン押下が ADC 値の有意な変化として現れず、現状は判定が不安定。pybricks の resistor ladder threshold 表 (`g_ladder_dev1_levels`) と PA1 周辺の wiring を再調査する必要がある。

## Bring-up シーケンス (btstack 主導)

1. NuttX ブート中: `stm32_bluetooth_initialize()` が nSHUTD LOW → slow clock 起動 → USART2 lower-half instantiate → nSHUTD 50ms LOW / HIGH / 150ms 待機 → `/dev/ttyBT` を register。
2. NSH で `btsensor start` 後 (daemon thread 内):
   - `btstack_run_loop_init` で NuttX run loop インスタンスを登録
   - `hci_init` + `hci_set_link_key_db` + `hci_set_chipset(btstack_chipset_cc256x_instance())`
   - `spp_server_init` で L2CAP/RFCOMM/SDP を構成（GAP は discoverable/connectable とも off）
   - `btsensor_tx_init` + `imu_sampler_init` で `/dev/uorb/sensor_imu0` の fd を data source として登録
   - `hci_power_control(HCI_POWER_ON)` → btstack state machine が HCI Reset → Read_Local_Version → Read_Local_Supported_Commands → HCI_VS_Update_Baud_Rate (0xFF36) → chipset init script ストリーミング (~40 chunks、~200 ms) → Read_BD_ADDR → Write_Page_Scan_* → HCI_STATE_WORKING
3. `HCI working, BD_ADDR ...` が syslog に出る (advertising はまだ off)。
4. `btsensor stop` 実行時は teardown FSM が GAP off → RFCOMM disconnect (cid != 0 のとき) → sampler/tx deinit → `hci_power_control(HCI_POWER_OFF)` → `BTSTACK_EVENT_STATE = HCI_STATE_OFF` 待ち → `btstack_run_loop_trigger_exit()` の順で進む。各 pending state は 3 秒 watchdog 付き。run loop 抜け後 daemon は `hci_close()` で btstack 状態をリセットしてから task 終了。

## SPP サービス仕様

- **Local name**: `SPIKE-BT-Sensor`
- **Class of Device**: `0x001F00` (Uncategorized)
- **Security**: SSP Just-Works (`SSP_IO_CAPABILITY_DISPLAY_YES_NO`), `LEVEL_2`
- **Service name**: `SPIKE IMU Stream`
- **RFCOMM channel**: 1
- **SDP UUID**: `0x1101` (SPP) + Profile Descriptor SPP v1.2

## RFCOMM ペイロード (IMU フレーム)

little-endian、Kconfig 既定の 1 フレーム = 18 byte header + 8 サンプル × 16 byte = 146 byte (1〜80 サンプルまで対応)。Issue #56 Commit E でワイヤフォーマットを刷新 (magic 変更、ヘッダに FSR / ODR、各サンプルに `ts_delta_us`) — 詳細は `docs/development/pc-receive-spp.md` を参照。

```c
struct spp_frame_hdr {                // 18 byte
    uint16_t magic;            // 0xB66B
    uint8_t  type;             // 0x01 = IMU
    uint8_t  sample_count;     // 1〜80
    uint16_t sample_rate_hz;   // 現在の ODR (Hz)
    uint16_t accel_fsr_g;      // 2 / 4 / 8 / 16
    uint16_t gyro_fsr_dps;     // 125 / 250 / 500 / 1000 / 2000
    uint16_t seq;              // フレーム単調連番
    uint32_t first_sample_ts_us; // CLOCK_BOOTTIME us の下位 32 bit
    uint16_t frame_len;        // = 18 + sample_count * 16
};

struct imu_sample {                   // 16 byte
    int16_t  ax, ay, az;       // LSM6DSL accel chip-frame 生 LSB
    int16_t  gx, gy, gz;       // LSM6DSL gyro  chip-frame 生 LSB
    uint32_t ts_delta_us;      // sample.ts - first_sample_ts_us
                               // (sample[0] = 0)
};
/* スケール係数は driver の起動時 FSR (デフォルト ±8g / ±2000 dps) で
 * 換算する: accel = raw * fsr_g * 9.80665 / 32768 [m/s^2]
 *           gyro  = raw * fsr_dps / 32768          [deg/s]
 * Issue #56 Commit E でフレームヘッダに FSR を埋め込む予定。
 */
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
CONFIG_APP_BTSENSOR_BATCH=8    # 1 RFCOMM frame に詰めるサンプル数 (既定)
CONFIG_APP_BTSENSOR_RING_DEPTH=8
```

Issue #47 で使っていた `CONFIG_WIRELESS_BLUETOOTH_HOST` / `CONFIG_NET_BLUETOOTH` / `CONFIG_BLUETOOTH_UART_GENERIC` / `CONFIG_BTSAK` / `CONFIG_NETDEV_LATEINIT` は Issue #52 Step A で削除した。

## NSH 操作

```
nsh> ls /dev/ttyBT
/dev/ttyBT

nsh> dmesg                                 # この defconfig には grep が無い
... BT: CC2564C powered, /dev/ttyBT ready
...

nsh> btsensor start                        # daemon を起動 (BT/IMU は共に off)
btsensor: started (pid 6)

nsh> dmesg                                 # バナーは syslog -> RAMLOG 経由
... btsensor: bringing up btstack on /dev/ttyBT
... btsensor: HCI working, BD_ADDR F8:2E:0C:A0:3E:64 — adv off ("SPIKE-BT-Sensor" hidden until BT button)

nsh> btsensor bt on                        # BT advertising を有効化 (BT ボタン短押しでも可)
OK
```

## ホスト adapter 互換性 (sustained streaming)

CC2564C 側は credit が残っていても、ホスト側 adapter のチップによっては `Number of Completed Packets` の HCI イベントが途中で途切れ、btstack が `RFCOMM_EVENT_CAN_SEND_NOW` を発火できなくなって stall することがある。観測済みの挙動:

| ホスト adapter | チップ | 30 秒 streaming テスト |
|----------------|-------|------------------------|
| Logitech 内蔵 USB ドングル | **MediaTek** | ~1.75 秒で停止 (`pending=1` で再開せず、~30 秒後に link supervision timeout で session close) |
| RPi 5 内蔵 | **Broadcom/Cypress** (CYW43455) | 30 秒継続 (sensor ODR 790 Hz / link ODR 662 Hz、~16% は ring overflow で drop) |

→ Hub 側コードは正しく動作しており、根本原因は MediaTek Classic BT firmware と CC2564C の interop。Hub アプリ側で回避するには rate-limit 等の改修が必要だが、現状は **adapter の選択** で回避することを推奨:

- ✅ **Broadcom/Cypress** (RPi 5 内蔵、Apple T2 等)
- ✅ **Intel** (AX200/AX210 等) — 未検証だが `iwlwifi` 系として一般的に良好
- ❌ **MediaTek** (多くの 安価 USB ドングルや Logitech "Unifying" 系)

検証方法は半自動テスト H-5 (`tests/test_bt_spp.py::test_bt_pc_pair_and_stream`) を実行する。`BTPROTO_RFCOMM` ソケットを直接開いて約 3 秒間フレームを受信し、Hub 側 `frames: sent` の伸びと `dropped` 比率 (≤25%) を検証する。H-5 を完走するアダプターは概ね問題ない。

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
