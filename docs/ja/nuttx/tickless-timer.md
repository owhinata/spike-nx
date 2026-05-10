# Tickless タイマー構成

SPIKE Prime Hub における NuttX tickless スケジューラのタイマー構成と、16 ビットタイマー使用時の動作について説明する。

## 構成

| 項目 | 設定 |
|------|------|
| モード | Tickless (`CONFIG_SCHED_TICKLESS=y`) |
| フリーランカウンタ | TIM9 (16 ビット, APB2 96 MHz) |
| インターバルタイマー | TIM7 (16 ビット, APB1 48 MHz) |
| tick 周期 | 10 µs (`CONFIG_USEC_PER_TICK=10`) |
| システム時刻型 | 64 ビット (`CONFIG_SYSTEM_TIME64=y`) |

### TIM2/TIM5 を使わない理由

STM32F413 で 32 ビットカウンタを持つタイマーは TIM2 と TIM5 のみだが、pybricks ファームウェアではそれぞれ ADC トリガーとバッテリー充電器 PWM に使用されている。将来 pybricks の機能を NuttX に移植する際の互換性を考慮し、TIM2/TIM5 は tickless タイマーに割り当てない。

## 16 ビットタイマーでのオーバーフロー処理

TIM9 は 16 ビット (0–65535) のため、100 kHz (10 µs/tick) で約 **655 ms** ごとにオーバーフローする。NuttX の tickless ドライバ (`arch/arm/src/stm32/stm32_tickless.c`) はこれを正しく処理している。

### システム時刻の取得 (`up_timer_gettime`)

```c
overflow = g_tickless.overflow;
counter  = STM32_TIM_GETCOUNTER(g_tickless.tch);
pending  = STM32_TIM_CHECKINT(g_tickless.tch, GTIM_SR_UIF);
verify   = STM32_TIM_GETCOUNTER(g_tickless.tch);

if (pending) {
    overflow++;
    counter = verify;
    g_tickless.overflow = overflow;
}

usec = ((((uint64_t)overflow << 16) + (uint64_t)counter) * USEC_PER_SEC) /
       g_tickless.frequency;
```

- `g_tickless.overflow` (32 ビット) がオーバーフロー回数を記録
- `(overflow << 16) + counter` で 48 ビット相当の連続時刻に拡張
- クリティカルセクション内でカウンタを 2 回読み取り、pending 割り込みがある場合はオーバーフロー後の値を採用することで、割り込みとのレースコンディションを回避

### インターバルタイマー (`up_timer_start`)

```c
DEBUGASSERT(period <= UINT16_MAX);
g_tickless.period = (uint16_t)(period + count);
```

- compare match 値は `(uint16_t)` にキャストされるため、16 ビットのラップアラウンドが自然に発生
- コード中のコメント: *"Rollover is fine, channel will trigger on the next period."*
- 1 回のインターバルで設定可能な最大遅延は約 655 ms だが、NuttX スケジューラはインターバルを必要に応じて再設定するため、`sleep()` 等の長い待ちでも問題なく動作する

### キャンセル時のロールオーバー補正 (`up_timer_cancel`)

```c
if (count > period) {
    period += UINT16_MAX;  /* Handle rollover */
}
```

カウンタが compare 値を超えている場合（ロールオーバー発生時）、残り時間の計算を補正する。

## Tickless wdog レース保護 (Issue #74 / #75 / #123)

Tickless wdog 経路は絶対 deadline (`next_tick`) を 16-bit lower-half timer 向けの相対 interval に変換する。負荷下では変換が `clock_systime_ticks()` を 2 回読み、その間にそれなりの作業が挟まることがあり、2 回目の read が `next_tick` を追い越して unsigned 減算が underflow した瞬間に `up_timer_start()` の `DEBUGASSERT(period <= UINT16_MAX)` が踏まれる (16-bit timer に対して数秒分の period が要求される形になるため)。

owhinata/nuttx fork (`f413-support-12.13.0` ブランチ) に 3 つの patch が積まれており、この変換を全段で保護している。

### Issue #74 — lower-half compare retry (`arch/arm/src/stm32/stm32_tickless.c`)

`up_timer_start()` は TIM9 `CNT` を読み、`compare = CNT + period` を計算して `CCR1` に書き込む。read と write の間に高優先 IRQ が走るとカウンタが既に compare を追い越している可能性があり、その場合は次の compare 一致がカウンタ 1 周分 (~655 ms) 後まで来ない。SETCOMPARE 直後にカウンタを再読みし、追い越されていれば `now + 1` で再 arm する有限ループ (最大 4 回) を入れている。

### Issue #75 — `wd_adjust_next_tick()` anchor + final guard (`sched/wdog/wdog.h`)

`wd_adjust_next_tick()` は元々 chunk interval の anchor を `g_wdexpired` に直接置いていたが、これは `clock_systime_ticks()` から遅れていることがある (例: scheduler が timer ISR 中に `interval=0` の round-robin event を挿入したケース)。anchor が古いままだと `next_tick = g_wdexpired + chunk` が `now` 以下に落ち、caller の unsigned `next_tick - now` が underflow する。anchor を `max(g_wdexpired, now)` に修正し、出口で `next_tick >= now + 1` を保証する final guard を入れることで、`wd_adjust_next_tick()` の返り値は関数内 `now` 比で必ず未来になる。

### Issue #123 — `wd_timer_start()` 外側クランプ (`sched/wdog/wdog.h`)

Issue #75 の guard は `wd_adjust_next_tick()` の **内側** の read に対してのみ有効。呼び出し元の `wd_timer_start()` は、返ってきた絶対 `next_tick` を `up_timer_tick_start()` 用の相対 delta に変換するため、もう一度 clock を読む必要がある。2 回の read の間に preempt が入ると 2 回目の `now` が `next_tick` を追い越し、再び unsigned 減算が wrap する — 内側 guard はこの外側 read には効かない。

通常は `enter_critical_section()` (BASEPRI=0x80) でこの race は塞がれるが、spike-nx の LUMP UART direct vector は NVIC priority 0x00 (BASEPRI より上) で動くため critical section を貫通して preempt できる。Issue #123 はこの結果として実機で踏まれた assert (`feat/122-capture` で ~80 分稼働後に 1 回、6 LUMP ポート + Bluetooth + capture トラフィック稼働中)。

修正は `wd_timer_start()` で再度 clock を読み、`clock_compare()` で delta をクランプする:

```c
clock_t now = clock_systime_ticks();
clock_t delta;

if (clock_compare(now, next_tick))
  {
    delta = next_tick - now;
#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
    if (delta > g_oneshot_maxticks)
      {
        delta = g_oneshot_maxticks;
      }
#endif
  }
else
  {
    delta = 1u;  /* deadline 過ぎ — lower-half retry に任せる */
  }

up_timer_tick_start(delta);
```

過ぎた deadline は 1-tick の re-arm に落ち (Issue #74 の retry loop が late-compare を吸収)、lower-half timer の表現可能範囲を超えた delta は最大値にクランプされる。どちらの経路でも `period <= UINT16_MAX` の assert は二度と踏まれない。

この fix はボード固有の HIPRI 配置に依存しない — clamp は「絶対 deadline → 相対 interval 変換が clock read を跨ぐ」あらゆる場面に効く一般保護として正当化できる。

## パフォーマンスカウンタ

`CONFIG_ARCH_PERF_EVENTS=y` により、STM32F413 の DWT CYCCNT (Data Watchpoint and Trace サイクルカウンタ) を使った高精度時間計測が有効になっている。

| 項目 | 値 |
|------|-----|
| カウンタ | DWT CYCCNT (32 ビット) |
| 周波数 | 96 MHz (SYSCLK) |
| 分解能 | 約 10.4 ns |
| オーバーフロー | 約 44.7 秒 |

### API

```c
#include <nuttx/clock.h>

clock_t t0 = perf_gettime();       /* カウンタ値を取得 */
unsigned long freq = perf_getfreq(); /* 周波数 (96000000) を取得 */

/* 経過時間の計算 */
clock_t t1 = perf_gettime();
uint64_t elapsed_ns = ((uint64_t)(t1 - t0) * 1000000000ULL) / freq;
```

- `perf_gettime()` — DWT CYCCNT の現在値を返す
- `perf_getfreq()` — カウンタ周波数 (Hz) を返す

32 ビットカウンタのため約 44.7 秒でオーバーフローする。長時間の計測には `clock_systime_ticks()` を使用すること。

## ostest wdog テストの WARNING について

ostest の wdog テストで以下のような WARNING が多数出力される:

```
WARNING: wdog latency ticks 65538 (> 10 may indicate timing error)
```

これは **テストの測定コードの問題** であり、OS の動作に影響はない。

### 原因

テストは `clock_systime_ticks()` の差分でレイテンシを計測する。`wdtest_rand` は 12,345 ns 以下のランダムな遅延を 1024 回繰り返すが、この中でクリティカルセクション外の `clock_systime_ticks()` 呼び出しタイミングと実際のコールバック発火タイミングにずれが生じ、大きなレイテンシ値が算出される。

### wdog は正常に動作している根拠

- `wdtest_assert(diff - delay_tick >= 0)` — コールバックが期待より早く発火した場合はアサートで停止するが、停止していない
- WARNING は「遅延が 10 tick 以上大きい」だけで、wdog が発火しなかったわけではない
- テスト全体が `wdog_test end...` まで正常に完走している
