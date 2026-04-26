# DMA ストリーム割当と IRQ 優先度

## 概要

SPIKE Prime Hub (STM32F413) で使用する DMA ストリーム割当と NVIC IRQ 優先度の設計。pybricks v3.6.1 の実装を基準とし、NuttX の IRQ 管理制約に適合させる。

## DMA ストリーム割当

RM0430 Table 27/28 (DMA request mapping) に基づく。

### DMA1

| Stream | Channel | ペリフェラル | 状態 | 優先度 |
|--------|---------|-------------|------|--------|
| S3 | Ch0 | **W25Q256 SPI2 RX** | ✅ 実装済 (NuttX SPI) | MEDIUM |
| S4 | Ch0 | **W25Q256 SPI2 TX** | ✅ 実装済 (NuttX SPI) | MEDIUM |
| S5 | Ch7 | **DAC1 CH1 (Sound)** | ✅ 実装済 | **HIGH** |
| S6 | Ch4 | **USART2 TX (CC2564C Bluetooth)** | 実装予定 (Issue #47) | VERY_HIGH |
| S7 | Ch6 | **USART2 RX (CC2564C Bluetooth)** | 実装予定 (Issue #47) | VERY_HIGH |

!!! note "USART2 RX の多重マッピング (F413 固有)"
    RM0430 Rev 9 Table 30 では USART2_RX が 2 箇所に定義される: S5/Ch4 と S7/Ch6。これは F413 固有の拡張 (CHSEL 4 ビット、§9.3.4 / Figure 24) で、16 チャネル mapping により多重マッピングが増えている。本プロジェクトは BT 専用に S7/Ch6 を使い、S5/Ch4 (= NuttX 既定 `DMAMAP_USART2_RX`) を残して将来用途に開けておく。

!!! note "Flash SPI2 DMA priority"
    pybricks は Flash SPI2 DMA を `DMA_PRIORITY_HIGH` で動かしているが、NuttX の `stm32_spi.c` は global `CONFIG_SPI_DMAPRIO` のみで per-bus 設定不可。HIGH に上げると TLC5955 SPI1 DMA も巻き込まれて Sound DMA1_S5 と DMA1 内 HIGH 3 本同時稼働 (S3/S4/S5) になり DAC underrun リスクが上がる。本プロジェクトは F4 デフォルトの MEDIUM (`DMA_SCR_PRIMED`) に据え置き、Sound 単独 HIGH を維持。pybricks 完全一致は将来 NuttX 上流貢献 (`stm32_spi.c` per-bus DMA priority) で対応予定。

### DMA2

| Stream | Channel | ペリフェラル | 状態 | 優先度 |
|--------|---------|-------------|------|--------|
| S0 | Ch0 | **ADC1** | ✅ 実装済 | **MEDIUM** |
| S2 | Ch3 | **TLC5955 SPI1 RX** | ✅ 実装済 (NuttX SPI) | LOW |
| S3 | Ch3 | **TLC5955 SPI1 TX** | ✅ 実装済 (NuttX SPI) | LOW |

### USB OTG FS

FIFO ベース。DMA 非使用。

## NVIC IRQ 優先度

### STM32F413 の NVIC 仕様

- 優先度ビット: 4 bit (16 レベル、0x00 が最高 / 0xF0 が最低)
- ステップ: `NVIC_SYSH_PRIORITY_STEP = 0x10`
- pybricks 優先度グループ: `NVIC_PRIORITYGROUP_4` (preempt 4bit / sub 0bit) — subpriority は実質無視
- NuttX デフォルト: `NVIC_SYSH_PRIORITY_DEFAULT = 0x80` (レベル 8)

### クリティカルセクション方式の違い

pybricks と NuttX では「割込マスクの範囲」が根本的に異なる。

| 項目 | pybricks | NuttX (現状 SPIKE Prime Hub) |
|---|---|---|
| API | `__disable_irq()` (`clock_stm32.c:31`, `reset_stm32.c:63` 等) | `up_irq_save()` (`armv7-m/irq.h`) |
| マスク機構 | **PRIMASK** — 全 IRQ を遮断 (NMI/HardFault を除く) | **BASEPRI = 0x80** — 優先度 `>= 0x80` の IRQ のみマスク |
| 効果 | どんな優先度の IRQ もクリティカル中は走らない | `< 0x80` の IRQ はクリティカル中も走れる (ただし NuttX API は呼べない) |
| SVCall | 0x00 (最高) — PRIMASK でも通る特例 | `NVIC_SYSH_SVCALL_PRIORITY = 0x70` — BASEPRI より上で貫通 |

!!! warning "BASEPRI の含意"
    NuttX で IRQ を `< 0x80` に設定すると、その ISR はクリティカルセクション中も走るが、**ISR 内で NuttX の任意の API を呼んではならない**。呼ぶ場合は ISR 内で NuttX 共有データへ触れず、signal/event だけを発行する限定 ISR (`CONFIG_ARCH_HIPRI_INTERRUPT` 参照) にする必要がある。

### 背景: なぜ BASEPRI 上の ISR で NuttX API が危険か

NuttX kernel は共有データ (scheduler ready/wait queue、semaphore waiter list、work queue など) を `enter_critical_section()` で保護する。これは内部で `BASEPRI = 0x80` にセットして優先度値 `>= 0x80` の IRQ をマスクする実装。

ISR が `< 0x80` にいると kernel のクリティカルセクションを**貫通**して preempt できる。その ISR 内で NuttX API を呼ぶと:

```
kernel thread          priority 0x60 ISR
--------------         -----------------
enter_critical()       (not yet firing)
BASEPRI = 0x80
  modifying wait
  queue list...      ← ISR at 0x60 preempts (0x60 < 0x80, not masked)
                      calls nxsem_post_slow()
                        enter_critical() (no-op, already 0x80)
                        modifies same wait queue ← RACE
                      ISR returns
  ...continues         ← list corrupted
leave_critical()
```

`nxsem_post` は以下 2 経路を持つ:

- **Fast path** — atomic `cmpxchg` で counter だけ増やす。待機スレッドが無い場合はこれだけ。ISR 安全 (priority 問わず)
- **Slow path** — `nxsem_post_slow()` が `enter_critical_section()` を取って **wait queue の linked list を操作する**。待機スレッドが 1 つでもあれば必ずこちらを通る

つまり受信待ちスレッドがある時点で slow path に入り、上記 race に発展する。

### NuttX ドライバ ISR の audit 結果

| IRQ | pybricks 値 | ISR パス | `nxsem_post` 呼び出し | BASEPRI 上可 |
|---|---|---|---|---|
| IMU I2C2 ER/EV | 0x30 | `stm32_i2c_isr` → `nxsem_post(&priv->sem_isr)` | あり (slow path 確実) | ❌ |
| Sound DMA1S5 | 0x40 | `stm32_dmainterrupt` (callback=NULL) | なし | ✅ |
| USB OTG FS | 0x60 | `stm32_usbinterrupt` → `cdcacm_rdcomplete` → `uart_wakeup` → `nxsem_post` | あり (slow path 確実) | ❌ |
| ADC DMA2S0 | 0x70 | `stm32_dmainterrupt` (callback=NULL) | なし | ✅ |
| TLC5955 SPI1 DMA | 0x70 | `spi_dmarx/txcallback` → `nxsem_post` | あり (slow path 確実) | ❌ |

USB/I2C/SPI は NuttX upstream ドライバに手を入れない限り BASEPRI 上に上げられない。

### 参考: `BASEPRI = 0` は「マスク無効」の特殊値

「BASEPRI=0 にしてクリティカルセクションを無効化すれば良い」という発想は**解にならない**。ARM Cortex-M では `BASEPRI = 0` は **「優先度マスク完全無効」の特殊値**で、どの IRQ も遮断しなくなる。これはクリティカルセクションが保護を一切提供しない状態で、NuttX kernel の共有データが任意の IRQ から破壊される。

### 参考: IRQ 優先度の表現は「絶対値」のみ (Cortex-M NVIC の制約)

Cortex-M NVIC は各 IRQ に絶対優先度値 (STM32F4 は上位 4bit、0x00–0xF0 の 16 段階) を書き込む方式で、「A は B より N 段上」のような**相対指定を NVIC 自身が計算する機構は存在しない**。priority grouping (PRIGROUP) も preempt/sub bit の割り方を決めるだけ。

ただし NuttX は以下の定数で**コード上は相対表現**が可能:

| 定数 | 値 | 意味 |
|---|---|---|
| `NVIC_SYSH_PRIORITY_MAX` | 0x00 | 最高優先度 |
| `NVIC_SYSH_PRIORITY_DEFAULT` | 0x80 | 中間値 (BASEPRI 境界でもある) |
| `NVIC_SYSH_PRIORITY_MIN` | 0xF0 | 最低優先度 |
| `NVIC_SYSH_PRIORITY_STEP` | 0x10 | 1 段階 |

現 `stm32_adc_dma.c` は `DEFAULT + 3 * STEP` と書いて 0xB0 を指定しており、「default から 3 段下」と読める。

### pybricks の IRQ 優先度一覧 (Prime Hub)

`HAL_NVIC_SetPriority(IRQ, preempt, sub)` — sub は実質 no-op。pybricks は PRIMASK で全 IRQ を遮断するため、絶対優先度はマイクロ秒レベルの応答差だけに効く。相対関係が設計意図を示す。

| preempt | NVIC 値 | IRQ | 設定箇所 |
|---|---|---|---|
| 0 | 0x00 | LUMP UART (UART4/5/7-10) | `uart_stm32f4_ll_irq.c:283` |
| 1 | 0x10 | Bluetooth UART TX DMA (sub=2) | `bluetooth_btstack_uart_block_stm32_hal.c:98` |
| 1 | 0x10 | Bluetooth UART RX DMA (sub=1) | 同上 :100 |
| 1 | 0x10 | Bluetooth USART2 (sub=0) | 同上 :102 |
| 3 | 0x30 | IMU I2C2_ER (sub=1) | `platform.c:188` |
| 3 | 0x30 | IMU I2C2_EV (sub=2) | `platform.c:190` |
| 3 | 0x30 | IMU EXTI4 (INT1, sub=3) | `platform.c:200` |
| **4** | **0x40** | **Sound DAC DMA (DMA1_S5)** | `sound_stm32_hal_dac.c:70` |
| 5 | 0x50 | Flash W25QXX SPI2 DMA (tx/rx) | `block_device_w25qxx_stm32.c:525, 542` |
| **6** | **0x60** | **USB OTG FS** | `platform.c:963` |
| 6 | 0x60 | USB VBUS (EXTI9_5, sub=1) | `platform.c:965` |
| 6 | 0x60 | Flash W25QXX SPI2 IRQ (sub=2) | `block_device_w25qxx_stm32.c:556` |
| 7 | 0x70 | ADC DMA (DMA2_S0) | `adc_stm32_hal.c:147` |
| 7 | 0x70 | TLC5955 LED SPI1+DMA (sub=0-2) | `pwm_tlc5955_stm32.c:197, 214, 234` |
| 15 | 0xF0 | SysTick (HAL tick) | `stm32f4xx_hal_conf.h:153` (`TICK_INT_PRIORITY`) |
| 15 | 0xF0 | RNG | `random_stm32_hal.c:54` |

### NuttX での IRQ 優先度設計 (採用: 選択肢 ε + Issue #50 精緻化)

pybricks の**相対優先順序**を 0x80–0xE0 の範囲 (BASEPRI 以下) に圧縮して配置する。全 IRQ が NuttX BASEPRI (0x80) 以上に収まるため、どの ISR からも `nxsem_post` 等の NuttX API を安全に呼べる。**0x80 は OS tick (TIM9) 専用**として peripheral との同格衝突を避け、0x90/0xA0 を LUMP UART (Issue #43) と Bluetooth UART (Issue #47) のために予約する。設定は `stm32_bringup.c` の先頭にまとめて集中管理。

| NVIC 値 | レベル | ペリフェラル | pybricks 対応 | 設定箇所 |
|---|---|---|---|---|
| 0x80 | 8 | TIM9 tickless tick (OS 専用) | (pybricks SysTick) | NuttX デフォルト (据置) |
| **0x90** | **9** | **LUMP UART (将来予約)** | base=0/1 | **Issue #43 で設定予定** |
| **0xA0** | **10** | **Bluetooth UART (USART2 + DMA1 S6/S7)** | base=1 | **Issue #47 で設定予定** |
| 0xB0 | 11 | IMU I2C2 EV/ER + EXTI4 | base=3 | `stm32_bringup.c` (step 6) |
| 0xC0 | 12 | Sound DAC DMA1_S5 | base=4 (HIGH) | `stm32_bringup.c` (step 5) |
| 0xD0 | 13 | W25Q256 DMA1_S3/S4 | base=5 | `stm32_bringup.c` (step 7) |
| 0xD0 | 13 | W25Q256 SPI2 IRQ | base=6 | `stm32_bringup.c` (step 7) |
| 0xD0 | 13 | USB OTG FS | base=6 | `stm32_bringup.c` (step 4) |
| 0xD0 | 13 | USB VBUS EXTI9_5 (将来) | base=6 | Issue #49 で設定予定 |
| 0xE0 | 14 | ADC DMA2_S0 | base=7 (MEDIUM) | `stm32_bringup.c` (step 3) |
| 0xE0 | 14 | TLC5955 SPI1 + DMA2_S2/S3 | base=7 (LOW) | `stm32_bringup.c` (step 2) |
| 0xE0 | 14 | BUTTON_USER EXTI0 (BT 制御ボタン) | n/a (NuttX 固有) | `stm32_bringup.c` (step 9, Issue #56) |
| 0xF0 | 15 | PendSV, SysTick | base=15 | `stm32_bringup.c` (step 1) |

!!! success "採用理由"
    - **OS tick (TIM9) 専用の 0x80 を peripheral と共有しない** — LUMP UART を 0x80 に置くと scheduler tick と同格になり、危険。0x90 を LUMP の最上位予約とする
    - 全 IRQ が BASEPRI 以上 ⇒ NuttX 管理の USB/I2C/SPI ドライバの ISR が `nxsem_post` 等を呼んでもレース無し
    - NuttX 本体への変更ゼロ (board 側 `stm32_bringup.c` のみで完結)
    - LUMP UART (Issue #43) と Bluetooth UART (Issue #47) の枠を空けてあるので、後続の実装で本表を再調整する必要がない

!!! note "Issue #50 での変更点"
    当初の ε 設計 (Issue #36) は 0xB0 に USB OTG FS と W25Q256 SPI2+DMA を同居させていた。Issue #50 で全体を 1 段降格して LUMP 用 0x90 / BT 用 0xA0 を空け、TIM9 (OS tick) に peripheral を相乗りさせない配置に変更。W25Q256 の DMA と SPI2 IRQ は引き続き同格 (0xD0) に据え置く — 分離した配置 (DMA=0xD0 / SPI2 IRQ=0xE0) を試したところ、Sound DMA + IMU I2C + Flash dd 並列時に USB CDC detach が発生したため、pybricks と同様 SPI2 の DMA と IRQ は同じ preempt level に置く。

!!! note "TIM9 を 0x80 に据え置き、peripheral を相乗りさせない理由"
    `CONFIG_SCHED_TICKLESS=y` + `CONFIG_STM32_TICKLESS_TIMER=9` により TIM9 が scheduling tick 源。scheduling 応答性を最優先するため ε 内で唯一 default 値に据え置く。**LUMP UART のような peripheral を同じ 0x80 に置くと OS tick と同格になり応答性に悪影響が出る可能性があるため、peripheral は 0x90 以下に配置する**。

### `CONFIG_ARCH_IRQPRIO` の実挙動

`nuttx/arch/arm/src/stm32/stm32_irq.c` を精査した結果、**`CONFIG_ARCH_IRQPRIO` はボード init で NVIC 初期化ロジックを変更しない**。この Kconfig は `up_prioritize_irq()` 関数の**定義**を有効化するだけで、`up_irqinitialize()` 内の NVIC 優先度レジスタ設定は `CONFIG_ARCH_IRQPRIO` の ON/OFF に関係なく全 IRQ を `NVIC_SYSH_PRIORITY_DEFAULT (0x80)` で初期化する。

ε 実装では defconfig に `CONFIG_ARCH_IRQPRIO=y` を追加し、`stm32_bringup.c` の `#ifdef CONFIG_ARCH_IRQPRIO` ブロック内で **up_prioritize_irq() を 10 回呼んで** 各 IRQ の NVIC 優先度レジスタを個別に書き換える。

### DMA 優先度 (DMA_SCR PL フィールド)

NVIC 優先度とは別に、DMA コントローラ内部のアービトレーション優先度。同一 DMA コントローラ内で複数ストリームが同時にリクエストした場合の調停に使われる。

| DMA 優先度 | ペリフェラル | 設定箇所 |
|---|---|---|
| HIGH | Sound DMA1_S5 | `stm32_sound.c` (DMA_SCR_PRIHI) |
| MEDIUM | ADC DMA2_S0 | `stm32_adc_dma.c` (DMA_SCR_PRIMED) |
| LOW | TLC5955 SPI1 | NuttX SPI driver (SPI_DMA_PRIO) |

## タイマー割当

| Timer | 用途 | 状態 | pybricks 用途 |
|---|---|---|---|
| TIM1 | Motor PWM (Port A/B) | 未実装 | 12 kHz PWM, PSC=8, ARR=1000 |
| TIM2 | ADC trigger (TRGO 1 kHz) | ✅ 実装済 | 同一 |
| TIM3 | Motor PWM (Port E/F) | 未実装 | 12 kHz PWM |
| TIM4 | Motor PWM (Port C/D) | 未実装 | 12 kHz PWM |
| TIM5 | Charger ISET PWM | defconfig 有、未実装 | 96 kHz, CH1 |
| TIM6 | DAC sample rate (TRGO) | ✅ 実装済 | 同一 |
| TIM8 | BT 32.768 kHz slow clock (CC2564C) | 実装予定 (Issue #47) | CH4 PWM (PSC=0, ARR=2929, CCR4=1465、32.764 kHz) |
| TIM9 | NuttX tickless timer | ✅ 実装済 | (pybricks では SysTick) |
| TIM12 | TLC5955 GSCLK | ✅ 実装済 | 同一 (CH2, ~8.7 MHz) |

## 解決: Issue #36 — 選択肢 ε を採用

`CONFIG_ARCH_IRQPRIO=y` を defconfig に追加すると、フル回帰 (`pytest -m "not slow and not interactive"`) で 7-8 テスト経過後に USB CDC/ACM デバイスが切断される問題 (Issue #36) が発生していた。調査の結果、原因は `stm32_adc_dma.c` が ADC DMA IRQ を `0xB0` に下げていた**構造**にあり、単純に「その優先度値が悪い」という話ではなかった。

### 調査の流れ

1. **切り分け step 1 (2026-04-18)**: `stm32_adc_dma.c` の `up_prioritize_irq()` だけを `#if 0` で無効化 → 30/30 pass。ADC DMA の優先度変更が直接トリガと確定。
2. **方針転換**: 単純な bisection ではなく、pybricks の設計 (Sound > USB > ADC の順位) を NuttX に移植する **選択肢 ε** を検討。
3. **ε 段階実装 (低優先度から順に、step 1–6)**:
   - step 1: PendSV + SysTick → 0xF0 ✅
   - step 2: TLC5955 SPI1 + DMA2S2/S3 → 0xD0 ✅
   - step 3: ADC DMA2S0 → 0xD0 ✅ (0xB0 では壊れたが 0xD0 (step 1+2 込み) では通る)
   - step 4: USB OTG FS → 0xB0 ✅
   - step 5: Sound DAC DMA1S5 → 0xA0 ✅
   - step 6: IMU I2C2 EV/ER + EXTI4 → 0x90 ✅
4. 全 step 完了後も 30/30 pass、CoreMark 170 固定、**Issue #36 解消**。

### 根本原因の考察

`0xB0` 単独では壊れ、`0xD0` + PendSV/SysTick/TLC5955 込みでは壊れないという事実から、#36 の原因は:

- **「ADC DMA が他の IRQ より低い」という構造そのもの** ではない (ε でも ADC は USB より低い)
- **PendSV (default 0x80) と peripheral IRQ との preempt 関係の何か**が絡んでいた可能性が高い
  - ε 前: ADC 0xB0 < PendSV 0x80 (= 他全部 0x80)。ADC ISR は常に PendSV に preempt され得る
  - ε 後: PendSV 0xF0 < 全 peripheral。PendSV は全 IRQ が片付いてから走る → Cortex-M の標準パターン
- 厳密な root cause 特定は今後の課題。ただし ε で安定動作しているので実運用上は解決。

### 採用しなかった選択肢 (参考記録)

対応策は複数あり、システム全体に影響する。以下は採用しなかった案の整理。

#### 選択肢 A: ADC DMA 優先度を default (0x80) に戻す (不採用)

`stm32_adc_dma.c` の `up_prioritize_irq()` 呼び出しを削除し、`CONFIG_ARCH_IRQPRIO=y` も不要にする。

- 長所: 最小変更、回帰リスクゼロ、現状 30/30 pass に戻る。
- 短所: **pybricks の相対優先度設計 (ADC=7 < Sound=4) を NuttX に移植できない**。将来 Motor / BT を実装して高優先度 IRQ が必要になったとき、ADC が全 IRQ と同じ 0x80 でいるのは設計として不整合。
- 判定: pybricks 設計思想を捨てることになるため不採用。ε で同等以上の相対順序が再現できた。

#### 選択肢 B: USB OTG FS を BASEPRI 直下に昇格 (例: 0x70) — 不採用

pybricks が USB を 0x60 (BASEPRI=0x80 より上) に置いていることに倣い、USB OTG FS IRQ の優先度を BASEPRI 直下に上げる。そうすれば ADC DMA が 0xB0 でも USB は preempt でき、USB 応答は改善される。

- 長所: pybricks の設計思想 (USB > ADC) を忠実に再現できる。
- 短所:
  - NuttX の USB CDC ISR (`stm32_otgfsdev.c` の `stm32_usbinterrupt`) は内部で `usbdev_*` API や work queue 投入を行う。これらが BASEPRI 貫通で呼ばれたとき排他が取れるか要検証。
  - 0x70 は `NVIC_SYSH_SVCALL_PRIORITY` と同じ値。SVCall と USB IRQ が同一優先度になることの影響 (context switch 中に USB が割り込んでの race) を検証する必要がある。
  - BASEPRI より上の IRQ は `CONFIG_ARCH_HIPRI_INTERRUPT` の領域で、NuttX のクリティカルセクションは効かない。USB CDC の TX/RX バッファ排他が既に spinlock/atomic でなければ破綻する可能性。
- 判定: **USB ドライバ内部の排他設計を精査してから判断**。軽々しく上げない。

#### 選択肢 C: NuttX 側の BASEPRI を引き上げる — 不採用

`NVIC_SYSH_DISABLE_PRIORITY` を 0xA0 などに引き上げれば、0xB0 < 0xA0 の関係でクリティカルセクション動作が変わる。

- 長所: アーキ全体で辻褄が合う。
- 短所: `nuttx/arch/arm/include/armv7-m/nvicpri.h` の変更になり、**NuttX 本体への侵襲的変更**。upstream と乖離する。プロジェクト方針 (nuttx submodule への変更は最小限) に反する。
- 判定: 却下。

#### 選択肢 D: `up_prioritize_irq()` による ADC DMA 降格をやめ、DMA PL (ハードアービトレーション) だけで優先度差を表現 — 不採用

`NVIC_SYSH_PRIORITY_DEFAULT` のままにし、ISR 応答の優先度差は諦めて DMA_SCR PL (Sound=HIGH / ADC=MEDIUM) だけで差を表現する。

- 長所: NVIC は触らない。選択肢 A と実質同じで IRQ 優先度の相対設計を諦める形。
- 短所: 選択肢 A と同じく将来の motor/BT 実装で再検討必須。
- 判定: 選択肢 A と同義。

#### 選択肢 ε: pybricks 相対順序を 0x80–0xF0 の範囲に圧縮 (**採用**)

pybricks の**絶対値** (0x00–0xF0 全域) ではなく**相対順序**だけを保存し、全 IRQ を BASEPRI (0x80) 以上に収める方式。全 IRQ が critical section で自動的にマスクされるので、USB/I2C/SPI など NuttX API を ISR で呼ぶドライバでも安全に動作する。

実装後の最終マッピングは上記「NuttX での IRQ 優先度設計 (採用: 選択肢 ε)」節を参照。

- 長所:
  - pybricks の**相対順序**を完全保存できる
  - 全 IRQ が BASEPRI 以上 ⇒ NuttX API 呼び放題、upstream ドライバそのまま使える
  - NuttX 本体変更ゼロ、board 側 (`stm32_bringup.c` の 1 ブロック) だけで完結
- 短所:
  - 使える幅が 0x80–0xF0 の 8 レベルしかない。現時点で 7 種なので余裕が少なく、将来 motor/BT を足す時に再設計になる可能性
  - 「ほぼ pybricks の PRIMASK 方式と同等の挙動」に近づくため、NuttX が BASEPRI 方式を選んだメリット (クリティカル中も高優先度 IRQ が貫通できる) は実質使えなくなる
- 判定: **採用**。実装時の段階的テスト (低優先度から順に、各 step で 30/30 pass 確認) により #36 が再発しないことを確認済み。

#### 選択肢 C1: NuttX の BASEPRI 値だけを引き下げる (`NVIC_SYSH_DISABLE_PRIORITY` を override) — 保留

BASEPRI を 0x30 程度に引き下げれば、pybricks 絶対値 (IMU 0x30、Sound 0x40、USB 0x60、ADC 0x70) が全て BASEPRI 以上に収まり、**pybricks の絶対値をそのまま移植できる**。

```c
/* nvicpri.h に 1 行パッチ、または board header で override */
#undef NVIC_SYSH_DISABLE_PRIORITY
#define NVIC_SYSH_DISABLE_PRIORITY 0x30
```

- 長所: pybricks 絶対値の完全移植が可能。全 IRQ で NuttX API 呼び放題
- 短所:
  - NuttX 本体 (`armv7-m/nvicpri.h`) への侵襲的変更
  - `NVIC_SYSH_SVCALL_PRIORITY` (現 0x70) との関係が崩れるので連動調整が必要
  - クリティカルセクション中に 0x30 以上の全 IRQ がマスクされる ⇒ レイテンシ面で現状より劣化
  - upstream と乖離し、`make nuttx-clean` 等で submodule 更新のたびに管理コスト発生

#### 選択肢 δ: NuttX critical section を PRIMASK 方式へ全面変更 — 棄却

`up_irq_save()` を BASEPRI ベースから `__disable_irq()` ベースに差し替える。pybricks と同じ「critical section は全 IRQ 遮断」モデル。

- 長所: 優先度値と ISR 安全性が完全分離される (pybricks 同等)
- 短所:
  - NuttX 本体への大きな侵襲 (`armv7-m/irq.h` 全面書き換え)
  - 全 IRQ がクリティカル中に止まる ⇒ リアルタイム応答性が現状より劣化
  - NuttX が BASEPRI 方式を採用している設計意図を覆す
- 判定: 棄却 (この Issue の範囲を超える)

### 将来の拡張時の留意

- **使える優先度レベル数は 8 (0x80–0xF0 までの `STEP=0x10` 刻み)**。現在 7 レベル使用済 (TIM9 / IMU / Sound / USB / ADC+TLC5955 / PendSV+SysTick)。将来 motor PWM や Bluetooth を足すと足りなくなる可能性がある。
- その場合の選択肢:
  - 同じ優先度に相乗り (例: ADC と TLC5955 のように)
  - 選択肢 C1 (BASEPRI を 0x30 等に引き下げ) を発動して 0x30–0xF0 の 13 レベルに広げる
- 新しい peripheral IRQ を追加する際は、pybricks の相対順序を確認して ε 枠内に配置 (`stm32_bringup.c` の該当ブロックに追記) する。

## ワークキュースレッド優先度 (HPWORK / LPWORK)

ISR が `work_queue(HPWORK, ...)` / `work_queue(LPWORK, ...)` で deferred 処理を投げる先のカーネルスレッド優先度。NVIC 優先度とは**完全に別軸**(NVIC は数値が小さいほど高、NuttX scheduler は数値が大きいほど高)で、両者の関係性は「ISR が defer したあとは、その work が thread 文脈で何優先度の隣人と CPU を取り合うか」だけ。

### 採用値

| Worker | NuttX PRI (1-255) | 設定 |
|---|---|---|
| `hpwork` | 224 (RR) | `CONFIG_SCHED_HPWORKPRIORITY=224` (NuttX 既定) |
| `lpwork` | **176** (RR) | `CONFIG_SCHED_LPWORKPRIORITY=176` (本プロジェクトで明示指定) |
| `lpwork` 上限 (PI 昇格時) | 176 | `CONFIG_SCHED_LPWORKPRIOMAX=176` (NuttX 既定) |

`SCHED_PRIORITY_DEFAULT = 100` (`<sys/types.h>`) なので、`nsh_main` / `imu` / `sound` / `coremark` 等の builtin app は基本 PRI=100 で走る。

### LPWORK を 100 → 176 に引き上げた理由

NuttX 既定の `LPWORKPRIORITY=100` だと user app と同じ優先度になり、

- ユーザアプリが CPU を使い切っている間 `lpwork` が `RR_INTERVAL=200ms` の time-slice に晒される
- BCD 検出のような時間非クリティカルな defer なら問題ないが、将来 ISR が時間敏感な作業を LPWORK へ流すと user-space CPU 負荷で遅延する

を避けたい。176 は `LPWORKPRIOMAX` の既定値で、HPWORK (224) と user app (100) の中間。これにより:

- LPWORK は user app を **常に preempt** する (低レイテンシな defer)
- HPWORK (224) は引き続き LPWORK を preempt できる (時間最優先の defer は HPWORK へ)
- 元々 PI で 176 まで昇格しうる設計だったので、ベースを 176 に固定しても挙動の上限は変わらない

### ドライバの defer 振り分け

| 発生源 | NVIC IRQ | defer 先 | 理由 |
|---|---|---|---|
| TIM9 (tickless tick) | 0x80 | — | tick handler 直で完了 |
| LSM6DSL DRDY (I2C2 / EXTI4) | 0x90 | **HPWORK** (224) | センサ DRDY → 直近の thread 経路で読出し |
| Sound DAC DMA1_S5 | 0xA0 | (DMA half/full cb 直, work 不経由) | リング buf 補充を ISR 内で完結 |
| USB OTG FS | 0xB0 | (driver 内部処理) | — |
| ADC DMA2_S0 | 0xD0 | (cb 直 → battery driver) | — |
| TLC5955 SPI1 + DMA2_S2/S3 | 0xD0 | **HPWORK** (224) | LED frame sync 2ms cadence を保つ |
| Battery charger poll | (timer) | **HPWORK** (224) | VBUS 状態の周期監視 |
| Battery charger BCD detect | (HPWORK 内で再 schedule) | **LPWORK** (176) | BCD は blocking I2C も含む heavy 処理を user 帯から切り離す |
| Power button monitor | (timer) | **HPWORK** (224) | 周期 poll |
| PendSV / SysTick | 0xF0 | — | — |

### 検証ポイント

- `nsh> ps` で `hpwork` PRI=224 / `lpwork` PRI=176 を確認
- LPWORK 引き上げは Issue #37 (BUILD_PROTECTED) のフォロー作業として `boards/spike-prime-hub/configs/usbnsh/defconfig` に追加(別構成 `nsh` defconfig には未反映 — 必要なら同様に追加)
- 既存の Issue #36 (NVIC) と Issue #37 (BUILD_PROTECTED) のいずれにも回帰なし。`pytest -m "not slow and not interactive"` で動作確認

## 将来の拡張

Motor / Bluetooth / Flash 実装時に追加すべき DMA/IRQ 設定は、pybricks の割当をそのまま踏襲する。pybricks では Bluetooth が最高優先度 (NVIC 1, DMA VERY_HIGH) だが、NuttX では BASEPRI 制約のため NVIC 0x80 に制限される可能性がある。この制約が BT 通信品質に影響するかは実装時に検証が必要。上記 Issue #36 の検討も同時期に再開する。

## 参照

- RM0430 Table 27: DMA1 request mapping
- RM0430 Table 28: DMA2 request mapping
- pybricks `lib/pbio/platform/prime_hub/platform.c`
- pybricks `lib/pbio/drv/sound/sound_stm32_hal_dac.c`
- pybricks `lib/pbio/drv/adc/adc_stm32_hal.c`
- pybricks `lib/pbio/drv/pwm/pwm_tlc5955_stm32.c`
- pybricks `lib/pbio/drv/bluetooth/bluetooth_btstack_uart_block_stm32_hal.c`
- pybricks `lib/pbio/drv/block_device/block_device_w25qxx_stm32.c`
- pybricks `lib/pbio/drv/uart/uart_stm32f4_ll_irq.c`
- pybricks `lib/pbio/drv/adc/adc_stm32_hal.c`
- NuttX `arch/arm/src/stm32/stm32_irq.c`
- NuttX `arch/arm/include/armv7-m/nvicpri.h`
