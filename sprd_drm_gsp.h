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

#ifndef _UAPI_SPRD_DRM_GSP_H_
#define _UAPI_SPRD_DRM_GSP_H_

#include <drm/drm.h>
#include "gsp_cfg.h"

#define DRM_SPRD_GSP_GET_CAPABILITY 0
#define DRM_SPRD_GSP_TRIGGER 1

struct drm_gsp_cfg_user {
  bool async;
  __u32 size;
  __u32 num;
  bool split;
  void *config;
};

struct drm_gsp_capability {
  __u32 size;
  void *cap;
};

#define DRM_IOCTL_SPRD_GSP_GET_CAPABILITY                                      \
  DRM_IOWR(DRM_COMMAND_BASE + DRM_SPRD_GSP_GET_CAPABILITY,                     \
           struct drm_gsp_capability)

#define DRM_IOCTL_SPRD_GSP_TRIGGER                                             \
  DRM_IOWR(DRM_COMMAND_BASE + DRM_SPRD_GSP_TRIGGER, struct drm_gsp_cfg_user)

#endif /* _UAPI_SPRD_DRM_GSP_H_ */
