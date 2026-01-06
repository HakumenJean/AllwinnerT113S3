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

#define DBG_TAG "romfs"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef RT_USING_DFS_ROMFS
#include <dfs_romfs.h>
#include <dfs_fs.h>

const struct romfs_dirent _root_dirent[] =
{
    {ROMFS_DIRENT_DIR, "sdcard", 0, 0},
    {ROMFS_DIRENT_DIR, "sda", 0, 0},
    {ROMFS_DIRENT_DIR, "sdb", 0, 0},
    {ROMFS_DIRENT_DIR, "sdc", 0, 0},
    {ROMFS_DIRENT_DIR, "sdd", 0, 0},
};

const struct romfs_dirent romfs_root =
{
    ROMFS_DIRENT_DIR, "/", (rt_uint8_t *)_root_dirent, sizeof(_root_dirent)/sizeof(_root_dirent[0])
};

int rt_hw_romfs_init(void)
{
    if(dfs_mount(RT_NULL, "/", "rom", 0, &romfs_root) < 0)
    {
        LOG_E("romfs init failed %d", rt_get_errno());
        return -1;
    }
    LOG_I("romfs init success");

    return 0;
}

INIT_EXPORT(rt_hw_romfs_init, "4.end");

#endif
