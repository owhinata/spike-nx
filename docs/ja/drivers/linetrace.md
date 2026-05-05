# linetrace (Issue #107)

LEGO color sensor から反射光サンプルを読み取りつつ `DRIVEBASE_DRIVE_FOREVER` を連続発行してライントレースを行う、NSH builtin のユーザー空間アプリ。pybricks 公式パターン (`pybricks/tests/motors/drivebase_line.py`) — `err = target - reflection(); drive(speed, err * Kp); wait(...)` をユーザースクリプト側で回す手法 — をそのまま spike-nx に持ち込んだもの。

## 1. なぜ別アプリで切り出すか

pybricks にはライントレース専用 API は存在しない。公式は MicroPython で 5 ms ループを書くだけのスクリプト例。spike-nx も同じ思想を採用: drivebase daemon (Issue #77) は 5 ms RT 制御 (台形軌道生成 / encoder/IMU fusion / PWM) に専念して line-trace 非依存のまま、ライントレースのポリシーは既存 `DRIVEBASE_DRIVE_FOREVER` ioctl の上に薄い user-space CLI として乗せる。

| 層 | 責務 |
|---|---|
| `apps/drivebase/` daemon (5 ms RT, prio 220) | 台形速度プロファイル・エンコーダ/IMU 融合・PWM 出力 |
| kernel `/dev/drivebase` chardev | コマンドリング・state ダブルバッファ・緊急停止 |
| **`apps/linetrace/` (このアプリ)** | カラーセンサ読み取り → turn rate 計算 → 100 Hz で `DRIVE_FOREVER` 再発行 |

daemon の cmd_ring (depth 8) が再発行を吸収する。新しい `DRIVE_FOREVER` envelope が来るたびに次 tick の目標値が上書きされ、5 ms tick は最新値を消費するだけ。新規 ioctl ABI は不要。

## 2. サブコマンド

```
linetrace cal                                   # 3 秒サンプリング
linetrace run <speed_mmps> <kp> [target] \      # 主コマンド
              [--max-turn dps] [--hz N]
linetrace                                       # usage 表示
```

デフォルト: `target=50`, `--max-turn=180`, `--hz=100`。`kp` は実数 (`linetrace run 100 1.5` 等)。

### 2.1 `linetrace cal`

`/dev/uorb/sensor_color` を CLAIM → SELECT(mode=1, REFLT) → 3 秒サンプリングし、その間に手で線の上 / 床の上をなぞる。観測 min / max / midpoint を表示。

```
nsh> linetrace cal
linetrace: sampling for 3000 ms — sweep the sensor over black/white
linetrace: cal 250 samples: black=4 white=72 midpoint=38
           suggested: linetrace run <speed_mmps> <kp> 38
```

### 2.2 `linetrace run`

1. `/dev/drivebase` を open し `DRIVEBASE_GET_STATE`。`active_command != DRIVEBASE_ACTIVE_NONE` (他の drive 系コマンドが進行中) なら refuse して exit。
2. `/dev/uorb/sensor_color` を open、CLAIM、SELECT(mode=1, REFLT)。
3. SIGINT handler を登録。
4. `--hz` (既定 100 Hz、`clock_nanosleep CLOCK_MONOTONIC TIMER_ABSTIME` で累積ドリフト 0) でループ:
   - color sensor を non-blocking に drain、最新サンプルを保持
   - `err = target - reflection`
   - `turn_dps = clamp(kp * err, ±max_turn)`
   - `ioctl(DRIVEBASE_DRIVE_FOREVER, {speed_mmps, turn_dps})`
5. SIGINT 受信時: `ioctl(DRIVEBASE_STOP, {COAST})` → fd close → exit。

## 3. チューニング

- `target`: ライン (黒) と床 (白) の中間反射光。`linetrace cal` で実測してから決める。
- `kp`: 1.0 から始めて発振するまで倍にしていき、発振したら 30 % 落とす。経験的に 1.5–4.0 が標準。
- `--max-turn`: 一瞬ラインを失った時に巨大な err が出ても吹っ飛ばないよう turn rate を頭打ちにする。56 mm 車輪のシャシで既定 180 dps は安全側。
- `--hz`: LPF2 カラーセンサの publish レートは ≈100 Hz なので、それ以上回しても同じサンプルを使い回すだけ。50 Hz 等に落とすと CPU は減るがレイテンシが増える。

## 4. 配線

- カラーセンサ: 任意ポート (LPF2 type 61 を auto-detect)。床から 10 mm 程度の高さで前向きに搭載。
- モータ: 標準 SPIKE Prime drivebase (port A/C を L、B/D を R)。`drivebase config <wheel_diameter_mm> <axle_track_mm>` を `linetrace run` の前に 1 度実行しておく。

## 5. Out of scope (別 Issue)

- D 項: 現状は P のみ。高速時の overshoot / 鳴きが見えたら追加する。
- ライン消失検出: 現状は最後のサンプルでループし続ける。pybricks にも対策は無い。必要なら N ms 新サンプル無し → coast の watchdog を後付け。
- gain schedule: 高速時は同じ閉ループ帯域を保つために kp を下げるのが定石だが現 CLI は定数。
- ジャンクション認識 (T 字 / 十字): 色判定 + ステートマシンが要る。
- IMU heading correction との合成: color 駆動と競合するので初版は color 単独。

## 6. 参考

- pybricks カノニカルパターン: `pybricks/tests/motors/drivebase_line.py`
- drivebase ABI: [drivebase.md](drivebase.md), `boards/spike-prime-hub/include/board_drivebase.h`
- color sensor (LPF2 type 61, REFLT mode): [sensor.md](sensor.md), `boards/spike-prime-hub/include/board_legosensor.h`
