 /* 
  * SPDX-FileCopyrightText: <FirstPublishedYear>-<LastUpdatedYear> Unisoc (Shanghai) Technologies Co., Ltd
  * SPDX-License-Identifier: GPL-2.0
  *
 * Copyright [First publish Year] [Last updated year] Unisoc (Shanghai) Technologies Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
  */
#define SPRD_MAP_IOCTRL_MAGIC        'o'

/**
 * DOC: MAP_USER_VIR
 *
 * Takes a pmem_info struct
 */
#define MAP_USER_VIR    _IOWR(SPRD_MAP_IOCTRL_MAGIC, 0, struct pmem_info)

struct pmem_info {
    unsigned long phy_addr;
    unsigned int phys_offset;
    size_t size;
};

