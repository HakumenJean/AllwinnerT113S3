/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#ifndef DRIVERS_DRV_CE_H_
#define DRIVERS_DRV_CE_H_

#define DES_MODE_ECB        (0)
#define DES_MODE_CBC        (1)
#define DES_MODE_CTR        (2)

#define DES_DIR_ENCRYPT     (0)
#define DES_DIR_DECRYPT     (1)

#define AES_MODE_ECB        (0)
#define AES_MODE_CBC        (1)
#define AES_MODE_CTR        (2)
#define AES_MODE_CTS        (3)
#define AES_MODE_OFB        (4)
#define AES_MODE_CFB        (5)

#define AES_DIR_ENCRYPT     (0)
#define AES_DIR_DECRYPT     (1)

#define HASH_METHOD_MD5     (16)
#define HASH_METHOD_SHA1    (17)
#define HASH_METHOD_SHA224  (18)
#define HASH_METHOD_SHA256  (19)
#define HASH_METHOD_SHA384  (20)
#define HASH_METHOD_SHA512  (21)

rt_err_t hash_md5_get(rt_uint8_t *buffer, rt_uint32_t buflen, rt_uint8_t *md5_dst);

#endif /* DRIVERS_DRV_CE_H_ */
