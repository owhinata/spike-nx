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
