/****************************************************************************
 * boards/stm32f413-discovery/kernel/stm32_userspace.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/wqueue.h>
#include <nuttx/userspace.h>

#if defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)

#ifndef CONFIG_NUTTX_USERSPACE
#  error "CONFIG_NUTTX_USERSPACE not defined"
#endif

#if CONFIG_NUTTX_USERSPACE != 0x08080000
#  error "CONFIG_NUTTX_USERSPACE must be 0x08080000 to match memory.ld"
#endif

static struct userspace_data_s g_userspace_data =
{
  .us_heap = &g_mmheap,
};

extern uint8_t _stext[];
extern uint8_t _etext[];
extern const uint8_t _eronly[];
extern uint8_t _sdata[];
extern uint8_t _edata[];
extern uint8_t _sbss[];
extern uint8_t _ebss[];

const struct userspace_s userspace locate_data(".userspace") =
{
  .us_entrypoint    = CONFIG_INIT_ENTRYPOINT,
  .us_textstart     = (uintptr_t)_stext,
  .us_textend       = (uintptr_t)_etext,
  .us_datasource    = (uintptr_t)_eronly,
  .us_datastart     = (uintptr_t)_sdata,
  .us_dataend       = (uintptr_t)_edata,
  .us_bssstart      = (uintptr_t)_sbss,
  .us_bssend        = (uintptr_t)_ebss,

  .us_data          = &g_userspace_data,

  .task_startup     = nxtask_startup,

  .signal_handler   = up_signal_handler,

#ifdef CONFIG_LIBC_USRWORK
  .work_usrstart    = work_usrstart,
#endif
};

#endif /* CONFIG_BUILD_PROTECTED && !__KERNEL__ */
