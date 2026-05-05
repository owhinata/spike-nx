# BT 経由 NSH シェル (`MODE SHELL`)

SPIKE Prime Hub の `btsensor` daemon は SPP RFCOMM 上で **NSH コンソール** を提供できる (Issue #108)。USB ケーブルを外したまま (バッテリ駆動の走行ロボなど) Hub の状態確認・コマンド投入を行うための仕組み。USB 側 NSH (`/dev/console`) は影響を受けない。

## 仕組み

```
PC ── BT/SPP/RFCOMM ── btsensor (userspace)
                          │
                          ├─ MODE_TELEMETRY (既定) — RFCOMM RX → btsensor_cmd_feed
                          │     IMU/sensor BUNDLE フレームを 100Hz 送出
                          │
                          ├─ MODE_SHELL_STARTING (一時的) — OK\n in-flight、stdin 送信禁止
                          │
                          └─ MODE_SHELL — RFCOMM ↔ /dev/btnsh_in/out FIFO ↔ NSH 子タスク
                                           btnsh_main (nsh_session 直叩き)
```

mode は telemetry と shell で **排他**。shell mode 中は telemetry pump が止まる (再開には明示的に `IMU ON` / `SENSOR ON`)。

## プロトコル

### shell mode に入る

PC は既存の SPP/RFCOMM 接続上で `MODE SHELL\n` を送る:

```
PC -> Hub:  MODE SHELL\n
Hub -> PC:  OK\n               <- ここまで MODE_SHELL_STARTING (stdin 送信禁止)
PC -> Hub:  ls /dev\n          <- OK 受信後は NSH stdin として解釈
Hub -> PC:  /dev/btnsh_in\n
            /dev/btnsh_out\n
            /dev/console\n
            ...
            nsh>\n
```

`OK\n` を受け取るまでの間に PC 側から bytes を送ると Hub 側で防御的に drop される (`syslog` warning)。**peer 契約として `OK\n` 受信まで stdin 送信禁止**。

### shell mode を出る

NSH 内で `exit` を入力するか、PC 側から RFCOMM 切断する:

```
PC -> Hub:  exit\n
Hub -> PC:  ... NSH の出力 ...
            <- NSH 子タスクが exit、stdout EOF、btsensor 内部で teardown
Hub -> PC:  READY\n            <- telemetry mode 復帰、コマンド受付可
PC -> Hub:  IMU ON\n           <- pump 再有効化 (自動再開はしない、明示的に)
Hub -> PC:  OK\n
```

`READY\n` は telemetry mode で再受付可能になったことを示す。

PC 側から RFCOMM 切断した場合、Hub は内部で `kill(SIGKILL)` + 子 reap + cleanup を走らせて MODE_TELEMETRY に戻る。`READY\n` は送らない (peer 不在のため)。

### `MODE SHELL` 失敗時

`shell_enter` の失敗 (`pipe()` / `open()` / `posix_spawn` エラー) や TX queue 満で `OK\n` enqueue 失敗時:

```
Hub -> PC:  ERR shell_<step> <errno>\n
```

または:

```
Hub -> PC:  ERR shell_no_buffer\n
Hub -> PC:  ERR shell_unavailable\n   <- daemon 起動時 mkfifo 失敗
```

**注意**: `MODE SHELL` を受信した時点で IMU/SENSOR pump は OFF にされる。`shell_enter` 失敗で `ERR` が返っても pump は OFF のまま。再有効化は明示的に `IMU ON\n` / `SENSOR ON\n`。

## USB NSH 側の操作

`btsensor mode` builtin で USB 側からも mode 切替可能。

```
nsh> btsensor mode shell
OK
nsh> # ここから USB NSH は使えるが、BT 側 RFCOMM peer がいなければ shell child は孤立
nsh> btsensor mode telemetry
OK
```

USB 側 `btsensor mode telemetry` は強制 teardown のエスケープハッチとして使える (BT 側で peer が固まった場合の復旧経路)。

## 既知制約

- **Ctrl-C / job control なし**: FIFO は tty ではなく、`isctty=false` で spawn しているため `TIOCSCTTY` 発行なし。長時間実行コマンドの中断は RFCOMM 切断のみ。
- **CRLF / ANSI 正規化なし**: peer terminal 設定で local-echo / CRLF 変換を行うこと (`screen` / `picocom` の `-c` 系オプション)。
- **stdin overflow**: 大量貼り付けで FIFO 4096B が飽和すると drop。間欠的タイピングなら問題なし。drop は Hub の `syslog` warn にのみ記録 (peer 通知なし)。
- **大量 stdout snapshot (`dmesg` / 長い `help` 等) は途中で truncate される** (Issue #109 follow-up): 真因は CC2564C controller の 4-slot ACL TX queue (Issue #54)。MTU いっぱいの RFCOMM frame を burst で投入すると controller queue が詰まり、永久 stall していた。Issue #109 close 時の "BlueZ credit refresh が遅い" 説明は誤り。現在の shell pump は `rfcomm_send()` を 256 B で cap し、5 ms BTstack timer で frame 間 throttle して controller queue を回し続ける。4 KB の TX coalescing buffer (default) を超えた分は reader pthread で drop されるが、**shell session 自体は生き続ける** — 切れた `dmesg` の後に `ps` / `ls /dev` / `free` / `md5` 等の短いコマンドはそのまま使える。1 byte も漏らしたくないなら `CONFIG_APP_BTSENSOR_SHELL_TX_BUF` を bump する (RAMLOG 16 KB に合わせるなら 16384)。
- **複数 SPP client 並行不可**: 既存 RFCOMM 構成と同じく単一 channel。
- **shell 中の telemetry frame 同時送信なし**: 排他設計のため。BUNDLE が必要なら `MODE TELEMETRY` で戻ること。

## トラブルシュート

| 症状 | 原因 | 対処 |
|---|---|---|
| `MODE SHELL` を送ったあと反応なし | telemetry pump (IMU/SENSOR) ON のまま | 先に `IMU OFF\n` / `SENSOR OFF\n` を送る |
| `ERR shell_unavailable` | daemon 起動時 mkfifo 失敗 (`CONFIG_PIPES`/`CONFIG_DEV_FIFO_SIZE` 未設定) | defconfig 確認、`btsensor stop` → `start` 再試行 |
| `OK\n` の後に NSH 出力が来ない | `nsh_session` の起動失敗 / 子タスクスタック不足 | `nsh> ps` で `btnsh` task の Stack used 確認、`CONFIG_APP_BTSENSOR_SHELL_NSH_STACK` を増やす |
| 大きな出力が途切れる | TX coalescing buffer overflow | `CONFIG_APP_BTSENSOR_SHELL_TX_BUF` を 8192 / 16384 に拡大 |
| `btsensor diag` で `hci blocked` が増える | CC2564C ACL TX queue stall 再発 (Issue #54) | `CONFIG_APP_BTSENSOR_SHELL_THROTTLE_MS` を 10/20 に上げる、または `CONFIG_APP_BTSENSOR_SHELL_TX_FRAME_BYTES` を 128 に下げる |
| 終了時に `READY\n` が来ない | RFCOMM 切断経由で抜けた | これは仕様。再接続後 `IMU ON` 等で telemetry mode 動作確認 |

## 関連設定 (Kconfig)

| Config | 既定 | 役割 |
|---|---|---|
| `CONFIG_APP_BTSENSOR_SHELL_MODE` | `n` (usbnsh defconfig は `y`) | shell mode 機能の有効化 |
| `CONFIG_APP_BTSENSOR_SHELL_TX_BUF` | 4096 | NSH stdout coalescing buffer (B) |
| `CONFIG_APP_BTSENSOR_SHELL_TX_FRAME_BYTES` | 256 | `rfcomm_send()` per-call cap (Issue #54 ACL stall 回避) |
| `CONFIG_APP_BTSENSOR_SHELL_THROTTLE_MS` | 5 | controller queue を回すための frame 間 throttle |
| `CONFIG_APP_BTSENSOR_SHELL_NSH_STACK` | 6144 | NSH 子タスクのスタック (B) |
| `CONFIG_APP_BTSENSOR_SHELL_READER_STACK` | 2048 | reader pthread のスタック (B) |
| `CONFIG_DEV_FIFO_SIZE` | 4096 (usbnsh defconfig) | FIFO バッファサイズ (B) |

## 診断 (`btsensor diag`)

USB NSH 側で `btsensor diag` を叩くと、BT 経由を通さずに hub 内部の TX path counters を読める。詰まった時の切り分けに使う。

| フィールド | 意味 |
|---|---|
| `reader: drops / drop_bytes` | `SHELL_TX_BUF` 越え。すべての出力が必要なら buffer を拡大 |
| `send: ok / no_credit / exceeds_mtu / other_err` | `rfcomm_send()` の戻り値別カウント |
| `hci: blocked / now / acl_free` | `blocked` が増え `acl_free` 低い → Issue #54 controller stall 再発のサイン |
| `ack: completed_events / packets` | peer 側の `Number_Of_Completed_Packets` event 受信数。経時で増えていなければ link 死亡 |
