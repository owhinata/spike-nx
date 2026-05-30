# センサーキャプチャ (`MODE CAPTURE`)

SPIKE Prime Hub の `apps/sensor` + `apps/btsensor` + `/dev/btcap` chardev は、センサー / RT 制御ループの内部データを **drop-free でキャプチャし BT 経由で PC に吸い上げる** 仕組みを提供する (Issue #122)。線追従 (linetrace) で Color sensor の MODE 比較 (Reflection vs RGBI 等) や走行ログを取りこぼしなく後追い解析するためのツールチェーン。

USB CDC ACM 経由で文字列ログを流すと printf jitter が RT 制御を乱すため、本機構は **同期 BT 転送のみ** を採用 (USB は経路に含めない)。

## 仕組み

```
apps/sensor (NSH)
   │   sensor color capture <ms>
   │   1) sensor uORB から duration_ms 分サンプル → heap buffer に積む
   │   2) capture_init/_write/_deinit で /dev/btcap に書き出す
   │              (reader engagement まで block — timeout なし)
   ▼
/dev/btcap (kernel chardev, board-side)
   │   pipe-style 1024B ring + IDLE/READY/ABORTED state machine
   │   ioctl: REGISTER / FINALIZE / ABORT / DRAIN_START/END /
   │          QUERY_STATE / GET_SESSION_META
   ▼
apps/btsensor MODE CAPTURE (userspace)
   │   3) /dev/btcap を drain、BTCS + meta + payload + (BTCE|BTAB) を SPP に流す
   │   4) drain 中は BUNDLE telemetry を pause、完了で復帰
   ▼
PC 側 CaptureViewer (host/CaptureViewer/, .NET 10)
       BTCS scan → 妥当性チェック → `.cap` ファイル化 → プロット
```

`apps/capture` は薄い export ライブラリ (`capture_init/_write/_deinit/_abort` の 4 関数)。`apps/sensor` がキャプチャ phase + export を回す唯一の v1 client。

## 用途別ワークフロー

### linetrace 用途 (Reflection vs RGBI 走行比較)

```
nsh> sensor color select 1                  # MODE 1 (Reflection)
nsh> sleep 1                                # 安定化
nsh> drivebase straight 200 &               # 走行 (background)
nsh> sensor color capture 3000 &            # 3 秒キャプチャ (mode 暗黙参照)
nsh> btsensor mode capture                  # ← BT 経由 PC へ drain 開始
nsh> wait                                   # background 終了確認

nsh> sensor color select 5                  # MODE 5 (RGBI) に切替
nsh> sleep 1
nsh> drivebase straight 200 &
nsh> sensor color capture 3000 &
nsh> btsensor mode capture                  # 2 回目 drain
nsh> wait
                                            # → CaptureViewer で 2 ファイル overlay
```

`sensor color capture` 単独ではデータ転送されない (writer が reader 待ちで block)。**必ず** `btsensor mode capture` (NSH) または PC から `MODE CAPTURE` (BT 経由 ASCII) で MODE 切替が必要。

データを諦める場合は `kill <apps/sensor の pid>` で強制終了 → kernel chardev の release fop が cleanup を担当 (timeout 機構なし)。

### linetrace ラップキャプチャ (Issue #166)

`linetrace` PID デーモンが走行中に 1 tick = 1 record でラップを記録し、
`linetrace cap export` で同じ `/dev/btcap` パイプラインを通して `.cap`
として吐き出す (schema `linetrace_lap_run` magic 0x0012、§capture-schemas)。

```
nsh> linetrace start
nsh> linetrace cap arm 2000        # ~20 s @ 100 Hz でバッファ確保
nsh> linetrace run 200 0.36 512    # ラップを走る
nsh>                               # ... 1 周走る ...
nsh> linetrace cap stop            # 凍結 (満杯になれば自動 stop)
nsh> linetrace brake
nsh> linetrace cap export &        # reader 待ちで block
nsh> btsensor mode capture         # 2nd session: /dev/btcap を drain
host$ cat /dev/rfcomm0 > lap.cap   # host 側で stream を保存
```

操作ポイント:

- `cap arm <n>` はモーションを起こさない。`arm` してから `run` でも、
  `run` してから `arm` でも、ARMED の間に流れた tick (idle / engaged
  問わず) が記録される。`cap_max` = 3449 records (64 KB / 19 B)。
- `brake` は走行中の capture を **凍結して保持** する (部分ラップも P0c に
  有用)。`cap stop` は明示的な「今すぐ凍結」verb。
- `linetrace stop` (デーモン終了) は未 export の capture を破棄する。
  `cap export` を `stop` より先に実行すること。
- `cap export` の後半は `sensor color capture` の export と同一 (同じ
  `/dev/btcap` 単一セッション契約、同じ「writer を kill してキャンセル」
  セマンティクス)。**btcap セッションは同時に 1 つだけ** なので、`sensor
  … capture` と `linetrace cap export` を同時に走らせることはできない。
- `kill -9 <export pid>` で強制終了した場合、次の `cap arm`/`cap export`/
  `cap status`/`cap abort` が dead exporter を回収して `idle` に戻す
  (lazy reaper)。明示的に捨てたいときは `cap abort`。

### キャプチャ phase の挙動

`sensor <class> capture [duration_ms]` (default 1000):

1. uORB topic を open し最初の sample で **暗黙の mode** を学習。`(class, mode_id) → schema` が静的テーブルに登録されていなければ `-ENOENT` で打ち切り (`select` 必須)。
2. duration_ms 分 / `CONFIG_APP_CAPTURE_MAX_HEAP_BYTES` 上限 (既定 64 KiB) のどちらか短いほうまで heap buffer に積む。途中で MODE が変わると `-EILSEQ`。
3. `capture_init` で `/dev/btcap` に session を REGISTER。
4. `capture_write` で全バイトを書き込む。chardev が pipe-style なので reader (`btsensor mode capture`) が DRAIN_START を打つまで write が block。
5. `capture_deinit` で FINALIZE → drain 完了まで block → IDLE 復帰。

SIGINT (Ctrl-C) / SIGTERM はハンドラがフラグを立て、loop 内で観測されたら capture phase を中断、export phase は `-ECANCELED`。`kill -9` (SIGKILL) は kernel default action で task termination → release fop で chardev cleanup。

### MODE CAPTURE 切替

`btsensor mode capture` (NSH) または PC からの `MODE CAPTURE\n` (BT 経由):

1. `/dev/btcap` を `O_RDONLY|O_NONBLOCK` で open。session が無ければ `ENOENT` で即返し (`btsensor: no capture session in flight`)。
2. session 検出時、BUNDLE emitter (IMU / sensor) を pause し data source を btstack run loop に attach。
3. 第 1 frame として **BTCS (4B) + meta (40B = u16 schema_magic + u16 reserved + u32 total_bytes + char[32] name)** を送出。
4. chardev から 256 B chunk で read → RFCOMM 送信 を 5 ms throttle (`CAP_TX_THROTTLE_MS`) で繰返し。`btsensor_tx` ring が満杯なら read 自体を控えて writer に back-pressure を返す (NFR-9 lossless paced sender)。
5. EOF を見たら kernel state を query。`READY` → BTCE (4B) を送出 (clean end)、`ABORTED` → BTAB (4B) を送出 (truncated)。
6. DRAIN_END で IDLE に戻し、BUNDLE emitter を **元の状態に** 戻して MODE TELEMETRY 復帰。

CAPTURE 中の `btsensor` は telemetry / ASCII reply を発行しない (排他)。

## PC 側受信

### 現状 (CaptureViewer.App 未実装)

raw 受信は `cat /dev/rfcomm0` で可能。**ただし RFCOMM デフォルトは tty なので line discipline が ^C/^Q/^S/^D 等を握りつぶす** (binary 中の制御バイトが消える)。`cat` 前に必ず raw 化:

```bash
stty -F /dev/rfcomm0 raw -echo -icanon -ixon -ixoff -opost min 1 time 0
cat /dev/rfcomm0 > capture.bin &
# Hub 側で sensor color capture & btsensor mode capture
# → BTCS + meta + payload + BTCE が capture.bin に届く
```

`xxd capture.bin | head` で先頭 4 バイトが `42 54 43 53` (`"BTCS"`) になっていれば成功。

### `host/CaptureViewer/` (.NET 10) のパース

`CaptureViewer.Core` を直接使う場合:

```csharp
using CaptureViewer.Core.Capture;
using CaptureViewer.Core.Generated;

// .cap ファイルを open
var cap = CaptureFile.Open("capture.cap");
Console.WriteLine($"{cap.SchemaName} {cap.RecordCount} records");

// schema 既知ならコード生成された Parse() で typed access
foreach (var i in Enumerable.Range(0, cap.RecordCount))
{
    var rec = SchemaColorReflectionRun.Parse(cap.Records(i).Span);
    Console.WriteLine($"ts={rec.ts_us}us refl={rec.reflection_pct}%");
}

// 連続 BT byte stream から session を切り出す場合
SessionScanner.TryScan(buffer.Span,
    magic => KnownSchemas.ByMagic.ContainsKey(magic),
    out var scan);
```

詳細は [スキーマリファレンス](capture-schemas.md) と `host/CaptureViewer/` のソースコードを参照。

## ワイヤーフォーマット

BT 経由で 1 session を運ぶ全バイト列:

```
+----------+--------------+------------------+----------+
| BTCS     | meta (40B)   | payload (.cap)   | BTCE/BTAB|
| 4B ASCII | u16 + u16    | u32 magic "CAPB" | 4B ASCII |
| "BTCS"   | + u32        | + 60B header     |          |
|          | + char[32]   | + N*48B fields   |          |
|          |              | + records        |          |
+----------+--------------+------------------+----------+
```

`payload` の中身は `apps/capture/include/capture_format.h` の `capture_file_header_s` (64 B) → `capture_field_desc_s[]` (48 B 各) → records。すべて little-endian。

## トラブルシュート

| 症状 | 原因 | 対処 |
|---|---|---|
| `capture: no schema for class=color mode=N` | (class, mode) → schema が未登録 | `apps/sensor/sensor_main.c` の `g_capture_schemas[]` に追加するか、対応 schema ヘッダを `apps/capture/include/` に追加 |
| `capture: no sample within Nms` | uORB topic が無発行 (ポート未接続 / `select` 未実行) | センサ接続確認、`sensor <class> select <mode>` 後に再実行 |
| `btsensor: no capture session in flight` | `btsensor mode capture` が writer 不在で起動 | `sensor <class> capture` を別 NSH session または `&` で先に走らせる |
| capture phase は完了するが BTCE が来ない | RFCOMM 切断 / pairing 喪失 / 5 ms throttle 上で long stall | `btsensor diag` で controller state 確認、PC 再 pair |
| `btsensor status` で `dropped_full > 0` | back-pressure 退行 (commit 6a4740d 参照) | `apps/btsensor/btsensor_capture_mode.c` の `on_read` で `btsensor_tx_frame_ring_full()` 確認 |
| `dropped_oldest > 0` (CAPTURE のみで) | `btsensor_tx_try_enqueue_frame` が CAPTURE chunk に対して drop-oldest 経路に入っている | これも上記と同じ調査経路 (CAPTURE chunk は drop-oldest を踏まないはず) |
| `cat /dev/rfcomm0` で session の途中までしか届かない | tty line discipline が制御バイトを除去 | `stty -F /dev/rfcomm0 raw -echo -icanon -ixon -ixoff -opost min 1 time 0` |

## 関連設定 (Kconfig)

| Config | 既定 | 役割 |
|---|---|---|
| `CONFIG_APP_CAPTURE` | usbnsh defconfig: `y` | userspace capture lib + `sensor capture` verb |
| `CONFIG_APP_CAPTURE_MAX_HANDLES` | 1 | 同時 in-flight session 数 (v1 は 1 固定) |
| `CONFIG_APP_CAPTURE_MAX_HEAP_BYTES` | 65536 | `apps/sensor` 1 セッションあたりの heap buffer 上限 |
| `CONFIG_APP_CAPTURE_BTCAP_RING_BYTES` | 1024 | kernel chardev pipe ring サイズ |
| `CONFIG_BOARD_BTCAP_CHARDEV` | usbnsh defconfig: `y` | `/dev/btcap` board-local chardev driver |

タイムアウト系 Kconfig (`WRITE_DEADLINE_MS` / `WAIT_DRAIN_TIMEOUT_MS`) は **持たない**。reader 不在は user 側の運用問題として task kill で対処する設計。

## 自動テスト

`tests/test_capture.py` (Issue #122):

| ID | mark | 内容 |
|---|---|---|
| K-1 | normal | `/dev/btcap` 登録確認 |
| K-2 | normal | `sensor` usage に `capture` verb と drain hint |
| K-3 | normal | unmapped mode で ENOENT path |
| K-4 | normal | writer 不在の `btsensor mode capture` rejection |
| K-5 | interactive | full BT round-trip (Color sensor + BlueZ pair 必須) |

```bash
# smoke 4 件
.venv/bin/pytest tests/test_capture.py -m "not slow and not interactive" -D /dev/ttyACM0

# K-5 単独 (operator pair 確認あり)
.venv/bin/pytest tests/test_capture.py::test_capture_round_trip_via_rfcomm -D /dev/ttyACM0
```

PC 側 CaptureViewer.Core:

```bash
dotnet test host/CaptureViewer/CaptureViewer.slnx          # 15 件
```

## 関連 Issue / commit

- Issue #122 (本機構の親 Issue)
- Issue #109 / #110 (BT NSH SHELL の throttle / back-pressure 設計、本機構の lossless 送信が踏襲)
- Issue #54 (CC2564C ACL TX queue stall — 5 ms throttle の根拠)
- Issue #111 (rcS auto-launch — `btsensor` が boot 時に起動済の前提)
