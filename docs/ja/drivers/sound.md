# サウンドドライバ

SPIKE Prime Hub の内蔵スピーカーを NuttX から鳴らすためのボードローカルドライバ群。DAC1 CH1 から生成した波形をアンプ経由でスピーカーに流す。

## ハードウェア構成

| 信号 | ピン / 周辺機能 | 説明 |
|------|----------------|------|
| DAC 出力 | PA4 (analog, DAC1 OUT1) | 12 bit 左詰めで波形を出力 |
| アンプ enable | PC10 (push-pull, active high) | LOW でスピーカーミュート |
| DMA | DMA1 Stream 5 Channel 7 | `DAC1_DHR12L1` へ循環転送 |
| サンプルタイマ | TIM6 (basic timer) | TRGO = update、入力 96 MHz |
| 波形中心 | `0x8000` | 12 bit 左詰めの対称中央値 |
| 最大サンプルレート | 100 kHz | ドライバ側で検証 |

STM32F413 の NuttX 上流 Kconfig は `STM32_HAVE_DAC1` を select しておらず、`stm32f413xx_pinmap.h` も DAC1 OUT1 マクロを持たないため、以下を**ボードローカル**で補っている:

- RCC `DAC1EN` / `TIM6EN` / `DMA1EN` を直接 enable
- `GPIO_DAC1_OUT1_F413` を `GPIO_ANALOG | PORTA | PIN4` として board 側に定義

## レイヤ構成

```
apps/sound (NSH builtin)
  sound beep / tone / notes / volume / off / selftest
        |           |
        v           v
  /dev/pcm0   /dev/tone0
   (raw PCM)   (tune string)
        \         /
         v       v
  stm32_sound_play_pcm / stop_pcm / set_volume
    DAC1 / DMA1 S5 / TIM6 (register direct)
```

- **低レベル層 (`stm32_sound.c`)** — `stm32_sound_play_pcm(data, length, sample_rate)` / `stm32_sound_stop_pcm()` を提供。呼出側が `g_sound.lock` を保持した状態で呼ぶ前提。
- **`/dev/tone0` (`stm32_tone.c`)** — tune 文字列を受け取るカーネル内パーサ付き char device。
- **`/dev/pcm0` (`stm32_pcm.c`)** — 単一呼出 ABI の raw PCM char device (pybricks `pbdrv_sound_start` と意味等価)。
- **`apps/sound`** — 上記 2 デバイスを NSH から叩くためのユーティリティ。

低レベル層は pybricks `pbdrv_sound_start(data, length, sample_rate)` と 1:1 対応した薄いラッパーで、音楽/ノートの概念を持たない。

## 共有状態と排他制御

```c
struct sound_state_s
{
    mutex_t           lock;         /* board-wide HW 排他 */
    int               open_count;   /* /dev/tone0 + /dev/pcm0 合算 */
    FAR struct file  *owner;        /* 現在の再生オーナー */
    enum sound_mode_e mode;         /* IDLE / TONE / PCM */
    uint8_t           volume;       /* 0..100 */
    atomic_bool       stop_flag;    /* tone_write 中断要求 */
};
```

- **HW 操作は必ず `g_sound.lock` 配下で実行**。`stm32_sound_play_pcm` / `stm32_sound_stop_pcm` 自体は lock を取らない (caller 責務)。
- **`stop_flag`** は `atomic_bool` (memory_order_relaxed)。`tone_write` 先頭で false クリア、`pcm_write` / `TONEIOC_STOP` / 自オーナー `close()` が true を立てる。`pcm_write` はクリアしない (並行する tone の中断要求を打ち消さないため)。
- **`close()`** は所有権ベース: オーナーが自分なら stop + owner クリア、所有者でないが `open_count == 0` になった場合のみ念のため stop。それ以外は何もしない (fork/dup で事故らないため)。
- **`stm32_sound_stop_pcm()` は冪等**。既に停止済の HW に呼んでも no-op。

## 再生・停止シーケンス

**start** (`stm32_sound_play_pcm`):

1. critical section に入る
2. TIM6 `CEN = 0`
3. DMA1 Stream5 disable → `EN = 0` を待って HIFCR で flag clear
4. DMA1 S5 を再設定 (`PAR = DHR12L1`, `M0AR = data`, `NDTR = length`, `SCR = CHSEL7 | PRIHI | MSIZE16 | PSIZE16 | MINC | CIRC | DIR_M2P | EN`)
5. DAC1 CR: `DMAEN1 | TSEL1_TIM6 | TEN1 | EN1`
6. TIM6 `PSC = 0`, `ARR = 96000000 / sample_rate - 1`, `EGR = UG`, `CEN = 1`
7. critical section を出る
8. **2 ms セトリング待ち** (`nxsig_usleep`) ― DAC 出力が中央値に到達するのを待ってからアンプを有効化
9. AMP_EN = HIGH

**stop** (`stm32_sound_stop_pcm`):

1. AMP_EN = LOW ― 先にアンプを遮断してポップを防ぐ
2. critical section に入る
3. TIM6 `CEN = 0`
4. DMA1 S5 disable + flag clear
5. DAC1 `CR = 0`
6. critical section を出る

## `/dev/tone0` (tune 文字列)

### 構文

pybricks 互換のノート文字列を受け付ける:

```
[T<bpm>] [O<octave>] [L<fraction>] <note>... 

<note>  := (<letter>[#|b][<octave>]) | R
           [ / <fraction> ] [.] [_]

<letter> := C | D | E | F | G | A | B
<fraction> := 1 | 2 | 4 | 8 | 16 | 32
```

- 先頭ディレクティブで `T<bpm>` (10..400, デフォルト 120), `O<octave>` (2..8), `L<fraction>` を変更可。
- `R` は休符。
- `.` は付点 (1.5 倍)。
- `_` は legato (リリースギャップなし)。
- 区切りは空白 / カンマ / 改行。

### タイミング

- 1 分音符 = `240_000_000 / bpm` μs (= 4 × 60 秒 / bpm)
- 実音部 = `whole_us / fraction` × (1 + 0.5 if 付点) − release_us
- release_us = 実音長の 1/8 (legato なら 0)
- 各音の `nxsig_usleep` は 20 ms スライスでループし、各スライス冒頭で `stop_flag` をチェックする

### 使用例

```
nsh> echo "C4/4 D4/4 E4/4 F4/4 G4/2" > /dev/tone0
nsh> echo "T240 L8 C4 D4 E4 F4 G4 A4 B4 C5" > /dev/tone0
nsh> echo "D#5/8. F#5/16 A5/4_" > /dev/tone0   # 付点 + legato
nsh> echo "C4/4 R/4 G4/4" > /dev/tone0         # 休符
```

### 中断

- `Ctrl-C` (SIGINT) で `nxsig_usleep` が中断され `-EINTR` 返却
- `ioctl(fd, TONEIOC_STOP, 0)` で即座に停止
- 並行する `/dev/pcm0` write でも奪取停止

## `/dev/pcm0` (raw PCM ABI v1)

pybricks `pbdrv_sound_start(data, length, sample_rate)` と意味互換の単一呼出 ABI。`write()` 1 回で header + サンプル列を全部渡す。

### ヘッダ構造体

```c
#define PCM_WRITE_MAGIC    0x304D4350u  /* "PCM0" little-endian */
#define PCM_WRITE_VERSION  0x0001u

struct pcm_write_hdr_s
{
    uint32_t magic;         /* PCM_WRITE_MAGIC */
    uint16_t version;       /* ABI version, 現在 1 */
    uint16_t hdr_size;      /* sizeof(struct) 以上 (将来拡張用) */
    uint32_t flags;         /* 現状 0 のみ (予約) */
    uint32_t sample_rate;   /* Hz, [1000, 100000] */
    uint32_t sample_count;  /* ヘッダ直後の uint16_t サンプル数 */
};
```

| offset | size | field |
|-------:|-----:|-------|
| 0  | 4 | `magic` |
| 4  | 2 | `version` |
| 6  | 2 | `hdr_size` |
| 8  | 4 | `flags` |
| 12 | 4 | `sample_rate` |
| 16 | 4 | `sample_count` |
| **total** | **20** | |

### write() 検証順序

1. `nbytes >= sizeof(struct pcm_write_hdr_s)` ... `-EINVAL`
2. `version <= PCM_WRITE_VERSION` (将来 version の前方互換性) ... `-EINVAL`
3. `magic == PCM_WRITE_MAGIC` ... `-EINVAL`
4. `hdr_size >= sizeof(struct)` かつ `hdr_size <= nbytes` ... `-EINVAL`
5. `1000 <= sample_rate <= 100000` ... `-EINVAL`
6. `sample_count <= UINT32_MAX / 2` ... `-EOVERFLOW`
7. `payload_bytes == nbytes - hdr_size` ... `-EINVAL`
8. `payload_bytes <= CONFIG_SPIKE_SOUND_PCM_BUFSAMPLES * 2` ... `-E2BIG`

検証から `memcpy` → `stm32_sound_play_pcm` → `owner = filep` までが `g_sound.lock` 配下で atomic に実行される。ユーザ空間ポインタは FLAT build 前提で `memcpy` する。PROTECTED build に移行する場合は `copyin()` 相当が必要。

### ループ再生前提

`/dev/pcm0` は **1 周期分の波形を循環再生**する前提のインターフェース (pybricks と同じ)。長い WAV を流す用途は非サポート。停止は `ioctl(TONEIOC_STOP, 0)` または `close()`。

### バッファサイズ

`CONFIG_SPIKE_SOUND_PCM_BUFSAMPLES` (Kconfig、デフォルト 1024 サンプル = 2 KB) で調整可能。範囲 [256, 8192]。

## ioctl (TONEIOC_*)

`/dev/tone0` と `/dev/pcm0` で共通。`arch/board/board_sound.h` で `_BOARDIOC()` を使って定義しており、上流の `AUDIOIOC_*` / `SNDIOC_*` と衝突しない。

| ioctl | arg | 説明 |
|-------|-----|------|
| `TONEIOC_VOLUME_SET` | `int` (0..100) | ボリュームを設定 (次の音から反映) |
| `TONEIOC_VOLUME_GET` | `int *` | 現在のボリュームを取得 |
| `TONEIOC_STOP` | 0 | 即座に再生停止 |

ボリュームは pybricks 互換の指数カーブで振幅スケーリングに使われる (0 = 無音, 100 = フルスケール 0x7fff)。

## `apps/sound` NSH ユーティリティ

`CONFIG_APP_SOUND=y` で有効化。

| コマンド | 説明 |
|----------|------|
| `sound beep [freq_hz] [dur_ms]` | square wave を組み立てて `/dev/pcm0` に write → usleep → `TONEIOC_STOP` (デフォルト 500 Hz / 200 ms) |
| `sound tone <freq> <dur_ms>` | `beep` の alias |
| `sound notes <tune>` | tune 文字列を `/dev/tone0` に write |
| `sound volume [0-100]` | `TONEIOC_VOLUME_SET/GET` |
| `sound off` | `TONEIOC_STOP` |
| `sound selftest` | 500 Hz 200 ms を `/dev/pcm0` 経由で再生 (bringup 動作確認代替) |

`sound beep` は raw PCM 経路の公式診断パスとして機能する (raw PCM はバイナリヘッダを要求するため `echo` から駆動不可)。

## pybricks との機能差分

本ドライバは pybricks `pbdrv_sound_start(data, length, sample_rate)` と 1:1 等価な低レベル層を起点に、その上に NuttX/NSH 向けの抽象を積んだ構成。pybricks の高レベル API (`Speaker.beep` / `Speaker.play_notes` / `Speaker.volume`) はすべて MicroPython 上の実装で、本プロジェクトはそれらに相当する機能をカーネル / NSH ビルトインとして再実装している。

### 共通する挙動

- **サウンドチェーン**: DAC1 CH1 (PA4) → 外部アンプ (PC10) → スピーカー。ハードウェア構成は pybricks と同一。
- **循環 DMA 再生**: 1 周期分の波形を DMA で繰り返し再生するモデル (バッファ = 1 周期、停止は明示呼出)。
- **波形生成**: 128〜256 サンプルの square wave、中央値中心。
- **ボリュームカーブ**: pybricks の `(10^(v/100) − 1) / 9 * INT16_MAX` を 10 % 刻みで事前計算したテーブル (両者とも同じ指数曲線)。
- **TIM6 をサンプルトリガに使用**: pybricks と同じ basic timer、他の汎用タイマを占有しない。
- **音量変更は次ノートから反映**: 現在鳴っている波形の振幅は変更しない。
- **ファイル再生は非対応**: pybricks も本ドライバも、長時間 WAV/RSF のストリーム再生はサポートしない。

### 本ドライバにあって pybricks にないもの

| 機能 | 本ドライバ | pybricks |
|------|------------|----------|
| NSH シェルからの tune 再生 | `echo "C4/4" > /dev/tone0` で直接駆動可能 | MicroPython 実行環境が必須 |
| カーネル内 tune パーサ | `stm32_tone.c` 内で `"C4/4 D#5/8. R/4 G4/4_"` を直接パース | MicroPython 側の Python コードでパース (カーネル非依存) |
| 単一呼出バイナリ PCM ABI | `struct pcm_write_hdr_s` (version/hdr_size/flags 付き) で `write()` 一発 | C 関数直接呼出し (API は内部のみ、ABI なし) |
| 将来拡張の version フィールド | `version` / `hdr_size` で前方互換の拡張が可能 | なし (内部 C API のみ) |
| マルチプロセス所有権追跡 | `filep` ベースの owner + `open_count` で奪取とクリーンアップを制御 | MicroPython ランタイム単一でそもそも複数プロセスを想定しない |
| 再生中の即時中断 | `nxsig_usleep` 20 ms スライス + `atomic_bool stop_flag` で `TONEIOC_STOP` / `Ctrl-C` / 並行 `pcm0 write()` による中断 | `beep()` は duration 経過までブロック (途中中断なし) |
| スタートポップ抑制 | DAC enable → 2 ms セトリング待ち → AMP_EN HIGH | `HAL_DAC_Start_DMA` 直後に AMP HIGH (遅延なし) |
| `close()` 自動停止 | 所有者一致時のみ停止、fork/dup でも事故らない | 該当概念なし (MicroPython 1 プロセス前提) |
| `TONEIOC_*` ioctl 空間 | `_BOARDIOC()` 経由でボードローカルに定義、NuttX 上流と衝突しない | 該当なし |
| NSH 診断コマンド | `sound beep`, `sound notes`, `sound volume`, `sound off`, `sound selftest` | `pybricks-micropython` スクリプトを実行する必要あり |

### pybricks にあって本ドライバにないもの

| 機能 | pybricks | 本ドライバ |
|------|----------|------------|
| MicroPython バインディング | `hub.speaker.beep(500, 100)` のような Python API | NuttX 環境のため Python ランタイム非搭載 |
| `beep()` のノンブロッキング補助 | `pb_type_Speaker_beep_test_completion` による非同期完了検知で協調マルチタスクに統合 | `write()` + `usleep` ベースの同期実装のみ (Ctrl-C / ioctl で中断は可) |
| `play_notes()` の Python イテレータ | 実行時に `notes_generator` として継続、別タスクと並行動作可 | `/dev/tone0` への write は単一呼出ブロッキング |
| タイマ常時動作 | `pbdrv_sound_init` で TIM6 を起動後、以降は停止せず循環 | 再生のたびに `TIM6.CEN` を 0→1 トグル |
| DMA バッファのゼロコピー | caller バッファ (MicroPython 側の静的 16 サンプル) をそのまま DMA に渡す | `/dev/pcm0` では user ポインタを kernel BSS に `memcpy` (NuttX の FLAT/PROTECTED セーフティ) |

### 互換性の保ち方

- **低レベル層の API は pybricks `pbdrv_sound_start` と 1:1 対応**しているため、将来の相互運用 (本ドライバ上で pybricks を動かす、または pybricks の tune generator を移植する) が比較的容易。
- **波形フォーマットと DAC レジスタアクセスも同じ**なので、pybricks 側のサンプルバッファ (`INT16_MAX` 中心) を `/dev/pcm0` に投げる変換は `0x8000` オフセットだけで済む。
- **`TONEIOC_*` は board-local ioctl 空間**なので、将来 pybricks 互換の上位 API を別 ioctl 空間 (例: `SNDIOC_*`) で追加しても衝突しない。

## 既知の制約

- **FLAT build のみ対応**。`CONFIG_BUILD_PROTECTED` では user pointer 直接 `memcpy` が不可なので `copyin()` 相当が必要。
- **PCM バッファは 1 周期ループ再生専用**。任意長 WAV を順次ストリームする用途は非サポート。
- **音量変更は次音から反映**。現在鳴っている音は変更前の振幅で鳴り続ける (pybricks と同じ)。
- **中断精度は約 20 ms スライス**。`stop_flag` ポーリングの粒度。

## 設計の経緯

### 最初の案: NuttX 標準 `tone_register()` + oneshot タイマ

当初は NuttX 標準の `drivers/audio/tone.c` の `tone_register()` を使い、DAC1 を PWM lower-half に見立てて `/dev/tone0` を登録、ノート長管理を NuttX の `oneshot_lowerhalf_s` (`stm32_oneshot` ベース) に任せる設計で着手した。

この方式は **F413 に使える oneshot タイマがない** ことが判明して破棄された:

- NuttX の `stm32_oneshot` は one-pulse mode を `CR1 = CEN | ARPE | OPM` で書き込むが、STM32F413 の `TIM10/11/13/14` は `CR1.OPM` ビットをハードウェアレベルでサポートしていない (RM0430 §19.5.1)。コンパイルは通るが実行時に発火しない。
- OPM 対応タイマ (`TIM1/2/3/4/5/8/9/12`) は pybricks 相当の機能を動かすために全て予約済みで空きはゼロ:
    - `TIM1/3/4` = モータ PWM (3 対)
    - `TIM2` = ADC DMA トリガ (1 kHz)
    - `TIM5` = バッテリ充電 ISET PWM
    - `TIM8` = Bluetooth 32 kHz クロック
    - `TIM9` = NuttX `CONFIG_STM32_TICKLESS_TIMER`
    - `TIM12` = TLC5955 GSCLK (9.6 MHz)
- したがって **oneshot タイマを増やさずに**ノート長管理を別の手段に移す必要があった。

関連 Issue: [#27](https://github.com/owhinata/spike-nx/issues/27), [#28](https://github.com/owhinata/spike-nx/issues/28), [#30](https://github.com/owhinata/spike-nx/issues/30)。

### 現在の設計への収束

pybricks と同じ方針で、カーネル層は「波形 + サンプルレートを受け取って循環 DMA で流すだけ」に徹し、ノート長管理は `nxsig_usleep` による同期ブロッキングに変更した。これで oneshot タイマ依存はゼロになり、サウンド全体が `TIM6` 1 本で動くようになった。

その後 Codex コードレビューを複数回回して、以下の設計項目を順に固めた:

1. **2 つの char device を並立させる構成** — tune 文字列用の `/dev/tone0` と、pybricks 互換の raw PCM 用 `/dev/pcm0` を両方提供し、`echo` による NSH 診断路と単一呼出バイナリ ABI の両方をカバーする。
2. **単一呼出 PCM ABI** (`struct pcm_write_hdr_s`) — `SETRATE` + `write()` のような順序依存を排除し、`version` / `hdr_size` フィールドで将来拡張の余地を残す。
3. **所有権ベースの `close()`** — `filep` をオーナーとして持ち、自分が再生中の場合のみ停止。fork/dup で別 fd を close した際に音が不意に止まる事故を防ぐ。
4. **1 ノート単位の mutex release + `atomic_bool stop_flag`** — `tone_write` はスライスごとに lock を離し、`TONEIOC_STOP` / Ctrl-C / 並行する `pcm0 write()` からの中断要求を 20 ms 以内に受け入れる。
5. **`stop_flag` のクリアルール限定** — `tone_write` 先頭でのみ false にクリアし、`pcm_write` はクリアしない (そうしないと実行中の tone の中断要求を打ち消してしまう)。
6. **start 時の順序: `TIM6 + DAC enable` → 2 ms セトリング待ち → `AMP_EN HIGH`** — DAC 出力が中央値に落ち着いてからアンプを立ち上げ、スタートポップを抑制する。

## 参考ソース

- [`boards/spike-prime-hub/src/stm32_sound.c`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/src/stm32_sound.c) — 低レベル PCM 再生
- [`boards/spike-prime-hub/src/stm32_tone.c`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/src/stm32_tone.c) — `/dev/tone0`
- [`boards/spike-prime-hub/src/stm32_pcm.c`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/src/stm32_pcm.c) — `/dev/pcm0`
- [`boards/spike-prime-hub/include/board_sound.h`](https://github.com/owhinata/spike-nx/blob/28cc9965f3aae82e7ec67707be53f3a90d18f7d8/boards/spike-prime-hub/include/board_sound.h) — 公開 ABI
- [`apps/sound/sound_main.c`](https://github.com/owhinata/spike-nx/blob/5f439034e32c2bc0e18e9c16f1c728739c0b47e1/apps/sound/sound_main.c) — NSH builtin
- pybricks リファレンス: `pybricks/lib/pbio/drv/sound/sound_stm32_hal_dac.c`
