# モーター制御ドライバ

## 1. H-Bridge ハードウェア構成

各 I/O ポートに H-bridge モータードライバがあり、2 本の PWM チャンネル (M1, M2) で方向制御する。

### ポート → タイマーチャンネル マッピング

| ポート | M1 ピン | M1 Timer/Ch | M2 ピン | M2 Timer/Ch | AF |
|---|---|---|---|---|---|
| A | PE9 | TIM1 CH1 | PE11 | TIM1 CH2 | AF1 |
| B | PE13 | TIM1 CH3 | PE14 | TIM1 CH4 | AF1 |
| C | PB6 | TIM4 CH1 | PB7 | TIM4 CH2 | AF2 |
| D | PB8 | TIM4 CH3 | PB9 | TIM4 CH4 | AF2 |
| E | PC6 | TIM3 CH1 | PC7 | TIM3 CH2 | AF2 |
| F | PC8 | TIM3 CH3 | PB1 | TIM3 CH4 | AF2 |

### タイマー設定

| タイマー | ポート | クロック | Prescaler | Period | PWM 周波数 |
|---|---|---|---|---|---|
| TIM1 | A, B | 96 MHz (APB2) | 8 | 1000 | 12 kHz |
| TIM3 | E, F | 96 MHz (APB1×2) | 8 | 1000 | 12 kHz |
| TIM4 | C, D | 96 MHz (APB1×2) | 8 | 1000 | 12 kHz |

**注**: APB1 タイマーは APB1 prescaler > 1 の場合クロックが 2 倍になる (48 MHz × 2 = 96 MHz)。
全チャンネル INVERT (反転 PWM) 設定。pybricks は 12 kHz を使用 (LEGO 公式は 1.2 kHz)。

---

## 2. H-Bridge 制御パターン

4 つの状態をピンの GPIO/AF モード切替で実現:

| 状態 | M1 (Pin1) | M2 (Pin2) | 効果 |
|---|---|---|---|
| **Coast** (惰性) | GPIO LOW | GPIO LOW | 両側 LOW、モーター自由回転 |
| **Brake** (制動) | GPIO HIGH | GPIO HIGH | 両側 HIGH、短絡制動 |
| **Forward** (正転) | PWM (AF) + duty | GPIO HIGH | M1 が PWM、M2 が HIGH |
| **Reverse** (逆転) | GPIO HIGH | PWM (AF) + duty | M1 が HIGH、M2 が PWM |

### Duty Cycle

- 範囲: [-1000, +1000] (±100%)
- 正値 → Forward、負値 → Reverse、0 → Brake
- Period = 1000 なので duty 値がそのまま CCR レジスタに書込まれる
- 反転 PWM のため、duty=0 → 常時 HIGH、duty=1000 → 常時 LOW

### 電圧補正

pybricks はバッテリー電圧変動を補正:
```
duty = target_voltage × MAX_DUTY / battery_voltage
```

---

## 3. モーターエンコーダ (LUMP UART 経由)

### データフロー

```
モーター内蔵エンコーダ → UART TX → Hub UART RX (LUMP)
  → legodev_pup_uart → pbio_tacho → pbio_servo (PID制御)
```

### エンコーダモード

**絶対エンコーダモーター** (SPIKE M/L/S Motor, Technic Angular Motor):
- Mode 3 (APOS): int16、1/10 度単位
- Mode 4 (CALIB): 絶対位置 + キャリブレーションデータ
- ソフトウェアで 360° 跨ぎを検出し回転数をトラッキング

**相対エンコーダモーター** (BOOST Interactive Motor):
- Mode 2 (POS): int32、累積度数

### PID 制御ループ

pybricks の PID 制御:

| パラメータ | 値 |
|---|---|
| ループレート | 5 ms (200 Hz) |
| 位置制御 | PID (比例 + 積分 + 微分) + アンチワインドアップ |
| 速度制御 | PI (速度偏差の P + 位置偏差の D) |
| 速度推定 | 100ms 窓 (20 サンプル) の微分器 |
| 状態推定 | ルーエンバーガーオブザーバー (角度, 速度, 電流) |
| ストール検出 | オブザーバーフィードバック電圧比 + 最小ストール時間 |

---

## 4. NuttX ドライバ設計

### NuttX の PWM ドライバ

NuttX は上位半分/下位半分のPWMドライバアーキテクチャを提供:
- `/dev/pwmN` キャラクタデバイス
- `ioctl(PWMIOC_SETCHARACTERISTICS)` で制御
- `CONFIG_PWM_MULTICHAN` でマルチチャンネル対応
- STM32 TIM1 の高機能タイマー (相補出力、デッドタイム) もサポート

**ただし NuttX にモーター制御フレームワークは存在しない。**

### 推奨アーキテクチャ

```
/dev/legomotor[N]  (カスタムモータードライバ)
  ├─ H-bridge 制御 (GPIO/AF モード切替 + PWM duty 設定)
  ├─ エンコーダ読取り (LUMP UART 経由)
  └─ PID 制御ループ (オプション、アプリ層でも可)
```

### ioctl インターフェース案

```c
#define LEGOMOTOR_SET_DUTY      _LEGOMOTORIOC(0)  // int16: -1000〜+1000
#define LEGOMOTOR_COAST         _LEGOMOTORIOC(1)  // 惰性
#define LEGOMOTOR_BRAKE         _LEGOMOTORIOC(2)  // 制動
#define LEGOMOTOR_GET_POSITION  _LEGOMOTORIOC(3)  // int32: 度数
#define LEGOMOTOR_GET_SPEED     _LEGOMOTORIOC(4)  // int16: deg/s
#define LEGOMOTOR_GET_ABS_POS   _LEGOMOTORIOC(5)  // int16: 絶対位置
#define LEGOMOTOR_RESET_POS     _LEGOMOTORIOC(6)  // 位置リセット
```

### GPIO/AF モード切替の実装

pybricks の核心パターン: **ピンを GPIO モード (Digital HIGH/LOW) と AF モード (Timer PWM 出力) の間で動的に切替**。

NuttX での実装:
```c
// Forward: M1=PWM, M2=HIGH
stm32_configgpio(GPIO_TIM1_CH1OUT);                // M1 → AF1 (PWM)
stm32_configgpio(GPIO_OUTPUT | GPIO_OUTPUT_SET |    // M2 → GPIO HIGH
                 GPIO_PORTE | GPIO_PIN11);

// Coast: M1=LOW, M2=LOW
stm32_configgpio(GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |  // M1 → GPIO LOW
                 GPIO_PORTE | GPIO_PIN9);
stm32_configgpio(GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |  // M2 → GPIO LOW
                 GPIO_PORTE | GPIO_PIN11);
```

### 代替アプローチ: PWM のみで制御

GPIO/AF 切替を避ける方法:
- 反転 PWM で duty=0 → 常時 HIGH、duty=period → 常時 LOW
- Forward: M1=duty_N, M2=duty_0 (= GPIO HIGH と同等)
- Coast: M1=duty_period, M2=duty_period (= GPIO LOW と同等)

この方式なら PWM チャンネルを常時有効のままにでき、NuttX 標準 PWM ドライバとの親和性が高い。

---

## 5. PID 制御の実装方針

### 初期段階

- 単純な duty 制御のみ (`LEGOMOTOR_SET_DUTY`)
- エンコーダ値の読取り (`LEGOMOTOR_GET_POSITION`)
- PID はアプリケーション層で実装

### 将来

- カーネル空間での PID ループ (5ms ワークキュー)
- pybricks のルーエンバーガーオブザーバーを参考に実装
- `LEGOMOTOR_RUN_TO_POSITION`, `LEGOMOTOR_RUN_AT_SPEED` 等の高レベル ioctl
