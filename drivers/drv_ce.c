/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>

#include "board.h"
#include "drv_ce.h"

#define DBG_TAG  "CE"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef BSP_USING_CE

#include "ce_common.h"

rt_err_t hash_md5_get(rt_uint8_t *buffer, rt_uint32_t buflen, rt_uint8_t *md5_dst)
{
    rt_err_t ret;
    uint8_t *dst_data;
    crypto_hash_req_ctx_t *hash_ctx;
    uint32_t data_size = SHA_MAX_DIGEST_SIZE;

    hash_ctx = (crypto_hash_req_ctx_t *)rt_malloc_align(sizeof(crypto_hash_req_ctx_t), max(CE_ALIGN_SIZE, CACHELINE_LEN));
    if(hash_ctx == NULL) {
        LOG_E("hash md5 memory malloc failed");
        return RT_ENOMEM;
    }

    dst_data = (u8 *)rt_malloc_align(data_size, max(CE_ALIGN_SIZE, CACHELINE_LEN));
    if (dst_data == NULL) {
        rt_free_align(hash_ctx);
        LOG_E("malloc dst buffer fail\n");
        return RT_ENOMEM;
    }

    rt_memset(hash_ctx, 0x0, sizeof(crypto_hash_req_ctx_t));
    rt_memset(dst_data, 0x0, data_size);

    hash_ctx->src_buffer = buffer;
    hash_ctx->src_length = buflen;
    hash_ctx->dst_buffer = dst_data;
    hash_ctx->dst_length = MD5_DIGEST_SIZE;
    hash_ctx->type = HASH_METHOD_MD5;
    hash_ctx->md_size = 0;

    ret = do_hash_crypto(hash_ctx);
    if(ret == RT_EOK) {
        rt_memcpy(md5_dst, hash_ctx->dst_buffer, MD5_DIGEST_SIZE);
        LOG_I("md5 hash get success");
    }

    rt_free_align(hash_ctx);
    rt_free_align(dst_data);

    return ret;
}

int rt_hw_ce_init(void)
{
    sunxi_ce_init();

    LOG_I("ce init success");

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_ce_init);

#ifdef RT_USING_FINSH
#include <finsh.h>

typedef struct {
    uint32_t instruction;
    uint32_t length;
    uint8_t md5sum[16];
    uint32_t version;
    uint32_t reserved1;
}MD5_FILE_HEAD;

int cmd_md5(int argc, char **argv)
{
    MD5_FILE_HEAD *head;
    int filelen = 756;
    uint8_t buffer[1024] = { 0 };

    head = (MD5_FILE_HEAD *)buffer;
    head->instruction = 0xEA000006;
    head->length = 0x00001600;

    for(int i = 0; i < 16; i += 4) {
        head->md5sum[i + 0] = 0x39 + i * 0x5;
        head->md5sum[i + 1] = 0x6C;
        head->md5sum[i + 2] = 0x0A;
        head->md5sum[i + 3] = 0x5F;
    }

    head->version = (1 << 24) | (0 << 16) | (0 << 8) | (0 << 0);    // 1.0.0.0
    head->reserved1 = (head->length >> 7) & 0xA7;

    for(int i = sizeof(MD5_FILE_HEAD); i < filelen; i++)
    {
        buffer[i] = i;
    }

    filelen = (filelen + 511) & (~511);

    hash_md5_get(buffer, filelen, head->md5sum);

    rt_kprintf("md5 result: ");
    for(int j = 0; j < 128; j++)
    {
        if((j % 16) == 0) rt_kprintf("\r\n");
        rt_kprintf("%02X ", buffer[j]);

    }
    rt_kprintf("\r\n");

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_md5, md5_test, test md5);

#endif

#endif
