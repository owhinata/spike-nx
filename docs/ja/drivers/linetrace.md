# linetrace (Issue #107)

LEGO color sensor の RGB+I intensity チャネル (mode 5、INT16 第4チャネル、0..1024) を読み取りつつ `DRIVEBASE_DRIVE_FOREVER` を連続発行してライントレースを行う、NSH builtin のユーザー空間アプリ。pybricks 公式パターン (`pybricks/tests/motors/drivebase_line.py`) — `err = target - intensity(); drive(speed, err * Kp); wait(...)` をユーザースクリプト側で回す手法 — をそのまま spike-nx に持ち込んだもの。Issue #125 で入力源を mode 1 (REFLT、INT8 0..100) から mode 5 に切り替えた (intensity の方が量子化が細かく、Reflection でライン端に出ていた 1 LSB ジッタが消える)。

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

デフォルト: `target=512`, `--max-turn=180`, `--hz=100`。`kp` は実数 (`linetrace run 100 0.3` 等)。intensity モードでは err 絶対値が ~10× になるので、reflection 時代の `kp` 値の **1/10 程度から開始する** (例: 1.5 → 0.15)。

### 2.1 `linetrace cal`

`/dev/uorb/sensor_color` を CLAIM → SELECT(mode=5, RGB+I) → 第4 INT16 チャネル (intensity, 0..1024) を 3 秒サンプリングし、その間に手で線の上 / 床の上をなぞる。観測 min / max / midpoint を表示。

```
nsh> linetrace cal
linetrace: sampling for 3000 ms — sweep the sensor over dark/bright
linetrace: cal 250 samples: dark=42 bright=982 midpoint=512
           suggested: linetrace run <speed_mmps> <kp> 512
```

### 2.2 `linetrace run`

1. `/dev/drivebase` を open し `DRIVEBASE_GET_STATE`。`active_command != DRIVEBASE_ACTIVE_NONE` (他の drive 系コマンドが進行中) なら refuse して exit。
2. `/dev/uorb/sensor_color` を open、CLAIM、SELECT(mode=5, RGB+I)。サンプル読み取りは `mode_id == 5 && data_type == LUMP_DATA_INT16 && num_values >= 4 && len >= 8` でゲートし、短い DATA フレームで zero-fill された `i16[3]` を有効サンプル扱いしないようにする。
3. SIGINT handler を登録。
4. `--hz` (既定 100 Hz、`clock_nanosleep CLOCK_MONOTONIC TIMER_ABSTIME` で累積ドリフト 0) でループ:
   - color sensor を non-blocking に drain、最新 intensity 値 (`s.data.i16[3]`、`[0, 1024]` にクランプ) を保持
   - `err = target - intensity`
   - `turn_dps = clamp(kp * err, ±max_turn)`
   - `ioctl(DRIVEBASE_DRIVE_FOREVER, {speed_mmps, turn_dps})`
5. SIGINT 受信時: `ioctl(DRIVEBASE_STOP, {COAST})` → fd close → exit。

### 2.3 `linetrace target <N>` / `linetrace max_turn <DPS>` (Issue #119)

`run` を打たずに `target` / `max_turn` を更新するための単機能サブコマンド。`linetrace start` 直後の idle 状態でも、運用 target を反映した `last_err` を `linetrace status` で確認できるようにするため。走行中に `max_turn` を変えると anti-windup clamp 値が次 tick から再導出されるので live tuning にも使える。

```
linetrace target 512           # cal で求めた midpoint を反映
linetrace max_turn 120         # 出力上限を狭めてアグレッシブ抑制
```

範囲: `target ∈ [0, 1024]`, `max_turn ∈ [1, 1000]`。引数なし / 余剰引数 / 非整数は usage エラー (現在値は `linetrace status` で参照)。

### 2.4 `linetrace pidstat` (Issue #118)

PID ゲイン (Kp/Ki/Kd) を実機調整するための観測コマンド。drivebase の `get-state` (Issue #115) と同じスタイルで、ヘッダ行 + interval ごと 1 行ずつのストリーミング形式で PID 内部状態を出す。

```
linetrace pidstat                       # 1 行 snapshot
linetrace pidstat 5000                  # 5 秒間 1 Hz サンプル (= 5 行 + summary)
linetrace pidstat 5000 100              # 5 秒間 100 ms interval (= ~50 行 + summary)
```

既定 `interval_ms = 1000` (1 Hz)。短 interval は printf jitter が制御ループに乗るため、特別な理由がなければ 1000 ms 以上を推奨。`interval_ms < 1000/hz` (制御周期未満) は reject。

#### 出力列 (14 列)

```
 time_ms      iter intens     err   err_min   err_max  err_avg    zc    d_max    d_avg      i_acc  turn_max  turn_avg    sat
```

| 列 | 型 | 意味 |
|---|---|---|
| `time_ms` | snapshot | pidstat 開始からの経過時間 (グラフ横軸) |
| `iter` | snapshot | daemon の制御 tick カウンタ (uint32 表示、≈497 日で wrap) |
| `intens` | snapshot | カラーセンサ RGB+I の intensity チャネル (0–1024)。`%6d` データ幅に合わせた省略表記 |
| `err` | snapshot | `target - intens` |
| `err_min/max` | interval 集計 | 直前 interval (100 Hz 全 tick) の偏差 min/max。**aliasing 回避** |
| `err_avg` | interval 集計 | 直前 interval の **mean(\|err\|) × 10** (0.1 固定小数点)。IAE 系追従品質指標 |
| `zc` | interval 集計 | 直前 interval の zero-crossing 回数 (符号反転) |
| `d_max` | interval 集計 | max(\|d_term\|)。Kd 過大時のスパイク振幅 |
| `d_avg` | interval 集計 | mean(\|d_term\|)。`d_max/d_avg` 比でノイズ性質判定 |
| `i_acc` | snapshot | 積分蓄積 (積分は遅変動なので集計不要) |
| `turn_max/avg` | interval 集計 | max/mean(\|turn_dps\|)。出力振幅 |
| `sat` | 累積 delta | pidstat 開始からの飽和カウント (clamp が effect した tick 数) |

集計 7 列 (err_min/max/avg, d_max/avg, turn_max/avg) は `interval_tick_count == 0` (idle 開始直後) のとき `-` 表示。

#### summary 行

```
# pidstat: sat=N iter=B..E duration_ms=M reported_ticks=R expected=X
```

`reported_ticks` は印字済み interval の tick 合計、`expected = duration_ms × hz / 1000`。差分から daemon 停止 / 半端 interval 捨て / 実行 jitter / tick 抜けを切り分け可能。`duration_ms = 0` (snapshot mode) では出さない。

#### 運用上の注意

- **pidstat 実行中に `linetrace run` / `brake` を打たない**。1 interval 内で engaged 状態が変わると d_avg/turn_avg が薄まる
- `linetrace status` は瞬時値の追従に使い、PID 各項の時系列は pidstat で取る (#118 で `last_p_term`/`last_i_term`/`last_d_term`/`last_turn_dps` を `status` から削除し、代わりに `last_i_acc` を追加)

## 3. チューニング

PID ゲインの実機調整は `linetrace pidstat` の各列を見ながら進める:

| 症状 | 観測する列 | 対処 |
|---|---|---|
| 発振 (Kp 過大) | `zc` 増加 + `err_min/err_max` 振幅拡大 | Kp を下げる |
| 積分ワインドアップ (Ki 過大) | `i_acc` が anti-windup clamp 値に張り付く | Ki を下げる |
| D 項ノイズ (Kd 過大) | `d_max / d_avg` 比が大きい (バーストノイズ) | Kd を下げる or LPF 追加 |
| 出力飽和 (max_turn 不足) | `sat` 連続増加 + `turn_max == max_turn` | `--max-turn` を上げる |
| 追従品質比較 | `err_avg` (IAE 系) | ゲインセット間で「より小さい方が良い」 |

ホスト側で時系列をログに保存しグラフ化する例 (`#` summary を skip):

```bash
picocom -t '!' /dev/ttyACM0 | tee pidstat.log
# 別端末で:
awk '!/^#/ && NR>1 {print $1, $4}' pidstat.log | gnuplot -p -e \
    "plot '<cat' using 1:2 with lines title 'err'"
# 列 4=err、列 5=err_min、列 6=err_max、列 7=err_avg(×10)、列 8=zc、
# 列 9=d_max、列 10=d_avg、列 13=turn_avg、列 14=sat
```

### 3.1 基本パラメータ

- `target`: ライン (暗) と床 (明) の中間 intensity 値。`linetrace cal` で実測してから決める。
- `kp`: 0.1 から始めて発振するまで倍にしていき、発振したら 30 % 落とす。経験的に 0.15–0.4 が標準。reflection 時代 (0..100) の kp の **約 1/10** に相当 — err 絶対値が intensity (0..1024) で ~10× になるため。
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
- color sensor (LPF2 type 61, RGB+I mode 5): [sensor.md](sensor.md), `boards/spike-prime-hub/include/board_legosensor.h`
