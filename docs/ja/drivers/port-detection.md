# I/O ポートデバイス検出

## 1. 概要

SPIKE Prime Hub の 6 つの I/O ポートは、接続されたデバイス (モーター/センサー) を自動検出する。検出は 2 段階で行われる:

1. **パッシブ検出** (抵抗ベース, ~400ms): GPIO ピンの電圧パターンで非 UART デバイスを識別
2. **UART 検出** (LUMP プロトコル): パッシブ検出で不明な場合、UART 通信でスマートデバイスを識別

## 2. ハードウェア構成

### ポートピン構成

各ポートには 5 つの信号線がある:

| 信号 | コネクタピン | 機能 |
|---|---|---|
| gpio1 | Pin 5 (入力パス) | ID1 読み取り |
| gpio2 | Pin 6 (入力/出力) | ID2 読み取り/駆動 |
| uart_tx | Pin 5 (出力パス) | ID1 駆動 / UART TX |
| uart_rx | Pin 6 (入力パス) | ID2 読み取り / UART RX |
| uart_buf | -- | バッファ有効化 (Active Low) |

`uart_tx` と `gpio1` は同じ物理ピン (Pin 5) にバッファ経由で接続。`uart_rx` と `gpio2` も同様 (Pin 6)。

### ポート別ピンマッピング

| ポート | UART | uart_tx | uart_rx | gpio1 | gpio2 | uart_buf | AF |
|---|---|---|---|---|---|---|---|
| A | UART7 | PE8 | PE7 | PD7 | PD8 | PA10 | AF8 |
| B | UART4 | PD1 | PD0 | PD9 | PD10 | PA8 | AF11 |
| C | UART8 | PE1 | PE0 | PD11 | PE4 | PE5 | AF8 |
| D | UART5 | PC12 | PD2 | PC15 | PC14 | PB2 | AF8 |
| E | UART10 | PE3 | PE2 | PC13 | PE12 | PB5 | AF11 |
| F | UART9 | PD15 | PD14 | PC11 | PE6 | PC5 | AF11 |

**注意**: ポート E と F は UART10/UART9 を使用 (STM32F413 固有)。

### 電源制御

- **ポート VCC (3.3V)**: PA14 で全ポート共通制御
- 起動時は VCC OFF → デバイス検出開始時に ON
- GPIO のプルアップ/プルダウンは全て無効化 (正確な抵抗検出のため)

## 3. パッシブ検出ステートマシン (DCM)

### ID グループ分類

GPIO ピンの電圧パターンを 4 グループに分類:

| グループ | 値 | 状態 |
|---|---|---|
| GND | 0 | GND に短絡 |
| VCC | 1 | VCC (3.3V) に接続 |
| PULL_DOWN | 2 | プルダウン抵抗経由 |
| OPEN | 3 | フローティング (未接続) |

### 検出フロー (2ms ポーリング)

```
Step 1: ID1 を HIGH 駆動 (uart_tx=HIGH, uart_buf=LOW), ID2 を入力に設定
  | YIELD (2ms)
Step 2: ID2 を読み取り → prev_value. ID1 を LOW 駆動
  | YIELD
Step 3: ID2 を読み取り → cur_value
  |- (1→0): タッチセンサー検出 (TOUCH)
  |- (0→1): トレインポイント検出 (TPOINT)
  +- 変化なし → Step 4 へ
  |
Step 4-5: ID1 グループを判定 (VCC/GND/PULL_DOWN/OPEN)
  | YIELD
Step 6-8: ID2 側も同様にテスト
  |- ID1=OPEN かつ (1→0): 3_PART
  |- (0→1): EXPLOD
  +- 変化なし → Step 9 へ
  |
Step 9-10: 最終判定
  |- ID1_group < OPEN → lookup[ID1_group][ID2_group]
  +- ID1_group = OPEN → UNKNOWN_UART (UART 検出へ)
```

### 抵抗マトリクスルックアップテーブル

| | ID2=GND | ID2=VCC | ID2=PULL_DOWN |
|---|---|---|---|
| **ID1=GND** | POWER (4) | TURN (3) | LIGHT2 (10) |
| **ID1=VCC** | TRAIN (2) | LMOTOR (6) | LIGHT1 (9) |
| **ID1=PULL_DOWN** | MMOTOR (1) | XMOTOR (7) | LIGHT (8) |

- **ID1=OPEN**: UART デバイスのシグネチャ → UART 検出フェーズへ

### デバウンス

同一タイプが **20 回連続** で検出されて初めて確定 (`AFFIRMATIVE_MATCH_COUNT = 20`)。ポーリング 2ms x 約 10 YIELD/サイクル x 20 回 = **約 400ms** で確定。

## 4. UART 検出 (LUMP プロトコル)

### GPIO → UART モード切替

`UNKNOWN_UART` が確定すると、ピンを GPIO から UART AF モードに切り替え:

```c
// NuttX での切替
stm32_configgpio(GPIO_AF8 | GPIO_PORTE | GPIO_PIN7);   // Pin 6 → UART RX AF
stm32_configgpio(GPIO_AF8 | GPIO_PORTE | GPIO_PIN8);   // Pin 5 → UART TX AF
// バッファ有効化 (Active Low)
stm32_gpiowrite(uart_buf_pin, false);
```

### LUMP メッセージフォーマット

ヘッダバイト (1 byte):
```
Bit [7:6] = メッセージタイプ (SYS=0, CMD=1, INFO=2, DATA=3)
Bit [5:3] = ペイロードサイズ (0=1B, 1=2B, 2=4B, 3=8B, 4=16B, 5=32B)
Bit [2:0] = コマンド/モード番号 (0-7)
```

- **SYS**: ヘッダのみ (SYNC=0x00, NACK=0x02, ACK=0x04)
- **CMD/DATA**: ヘッダ + ペイロード + チェックサム (XOR)
- **INFO**: ヘッダ + info_type + ペイロード + チェックサム

### ハンドシェイクシーケンス

```
Phase 1: 速度ネゴシエーション
  Hub → Device: CMD SPEED 115200 (115200 baud)
  Device → Hub: ACK (成功) or タイムアウト (→ 2400 baud EV3 モード)

Phase 2: 同期
  Device → Hub: CMD TYPE <type_id> (デバイスタイプ識別)
  最大 10 回リトライ

Phase 3: モード情報取得
  Device → Hub:
    CMD MODES <num_modes>
    CMD SPEED <baud_rate>
    CMD VERSION <fw_ver> <hw_ver>
    各モードごとに:
      INFO NAME <name>
      INFO RAW/PCT/SI <min> <max>
      INFO FORMAT <num_values> <type> <digits> <decimals>
    SYS ACK (情報送信完了)

Phase 4: 確認応答 + ボーレート切替
  Hub → Device: SYS ACK
  10ms 待機
  Hub: 新ボーレートに切替 (通常 460800 baud)

Phase 5: 通常運用
  Hub → Device: SYS NACK (100ms ごとの keep-alive)
  Hub → Device: CMD SELECT <mode> (モード切替)
  Device → Hub: DATA <payload> (センサーデータ)
  600ms 無応答で切断検出
```

### LUMP 状態マシン

```
RESET → SYNC_TYPE → SYNC_MODES → SYNC_DATA → ACK → DATA
  ^                                                  |
  +--------- タイムアウト / エラー ------------------+
```

## 5. デバイスタイプ一覧

### パッシブデバイス (抵抗マトリクス検出)

| ID | 名前 | 説明 |
|---|---|---|
| 0 | NONE | 未接続 |
| 1 | LPF2_MMOTOR | WeDo 2.0 Medium Motor |
| 2 | LPF2_TRAIN | Train Motor |
| 5 | LPF2_TOUCH | Touch Sensor |
| 8 | LPF2_LIGHT | Powered Up Lights |

### UART デバイス (LUMP プロトコル検出)

| ID | 名前 |
|---|---|
| 29 | EV3 Color Sensor |
| 30 | EV3 Ultrasonic Sensor |
| 32 | EV3 Gyro Sensor |
| 33 | EV3 IR Sensor |
| 34 | WeDo 2.0 Tilt Sensor |
| 35 | WeDo 2.0 Motion Sensor |
| 37 | BOOST Color and Distance Sensor |
| 38 | BOOST Interactive Motor |
| 46 | Technic Large Motor |
| 47 | Technic XL Motor |
| 48 | SPIKE Medium Motor |
| 49 | SPIKE Large Motor |
| 61 | SPIKE Color Sensor |
| 62 | SPIKE Ultrasonic Sensor |
| 63 | SPIKE Force Sensor |
| 64 | Technic Color Light Matrix |
| 65 | SPIKE Small Motor |
| 75 | Technic Medium Angular Motor |
| 76 | Technic Large Angular Motor |

## 6. NuttX 実装上の課題

1. **UART 所有権**: 検出中は UART ピンを GPIO として使用。UART ドライバは起動時に初期化せず、デバイス検出後に動的初期化
2. **2ms タイミング**: `work_queue(HPWORK, ...)` で実現可能
3. **ボーレート切替**: LUMP プロトコルで 115200 → 460800 等の動的切替が必要
4. **ホットプラグ**: 検出 → UART 通信 → 切断検出 → 再検出のループを永続スレッドで実行

## 7. NuttX 実装ステータス (Issue #42 / #43 完了)

DCM (パッシブ検出 + UART ハンドオフフック) は Issue #42 で実装済。LUMP プロトコルエンジン本体は Issue #43 で実装済 — 詳細は [lump-protocol.md](lump-protocol.md)。

### 実装ファイル

- `boards/spike-prime-hub/include/board_legoport.h` — 公開 ABI (ioctl 番号、type enum、構造体)
- `boards/spike-prime-hub/src/stm32_legoport.c` — DCM ステートマシン + ハンドオフレジストリ
- `boards/spike-prime-hub/src/stm32_legoport_chardev.c` — `/dev/legoport[0-5]` chardev shim
- `apps/legoport/legoport_main.c` — `legoport list / info / wait / stats` CLI

### 実行モデル

- HPWORK 2 ms 周期で 1 つの yield 境界 = 1 ステート前進 (pybricks の `PT_YIELD(pt)` を忠実に再現)
- 非 yield 状態遷移は同一 invocation 内で fall-through (tick を消費しない)
- 経路長: TPOINT=3 yields, TOUCH=4, OPEN/PULL ×VCC/PULL_DOWN=10 (worst). debounce 20 連続一致 ≈ 400 ms (worst)。

### Type ID

公開 ABI は pybricks `PBDRV_LEGODEV_TYPE_ID_LPF2_*` の値を保存:

- 1=MMOTOR, 2=TRAIN, 3=TURN, 4=POWER, 5=TOUCH, 6=LMOTOR, 7=XMOTOR
- 8=LIGHT, 9=LIGHT1, 10=LIGHT2, 11=TPOINT, 12=EXPLOD, 13=3_PART
- **14=UNKNOWN_UART** (UART デバイス確定。LUMP が解決するまで latch)

### ハンドオフ API (#43 連携)

```c
int stm32_legoport_register_uart_handoff(int port, legoport_uart_handoff_cb_t cb, void *priv);
int stm32_legoport_release_uart(int port);
```

- `register_uart_handoff` 後、DCM が `UNKNOWN_UART` 確定時に CB 呼出。CB は `uart_tx`/`uart_rx` を AF に切替・`uart_buf` low・LUMP エンジン起動を担当。
- 保護: `handoff_generation` で stale 戻り値防止、`handoff_inflight` + drain 待ちで `priv` lifetime 保護。
- **オーナー契約**: CB が OK を返した後、#43 は **全 exit path で必ず `release_uart(port)` を呼ぶ**。DCM 側に watchdog 無し。リーク owner = ポート永久 latch (リブートまで復帰不可)。
- **CB 非再入**: CB 内から同一ポートの `register/release` を呼ぶと inflight drain で deadlock。

### `WAIT_CONNECT/DISCONNECT` セマンティクス

`event_counter` (uint32_t monotonic) と per-fd snapshot で edge を取りこぼし無く検出。同じ ioctl が単一 fd で連続発行されても missed-edge race にならない。

### UNOWNED 再スキャン

CB 未登録 or 失敗時は `LATCHED_UART_UNOWNED_IDLE` で 100 ms ごとにフルスキャン。連続 K=3 回 NONE で latch 解除 (disconnect 検出 latency ≈ 350 ms)。

## 8. 参考リソース

- [pybricks/technical-info UART Protocol](https://github.com/pybricks/technical-info/blob/master/uart-protocol.md)
- [ev3dev UART Sensor Protocol](http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html)
- `pybricks/lib/pbio/drv/legodev/legodev_pup.c` -- ポートレベルドライバ (DCM 移植元)
- `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c` -- LUMP UART プロトコル (1265 行、#43 で移植済)
