/**
 * @file    diskio.c
 * @brief   FatFS 底层设备对接 (仅 SD 卡)
 *
 * 基于厂商例程 diskio.c 改写, 差异:
 *   - 删除外部 NORFLASH(卷1) 分支: 本工程无该器件
 *   - 删除厂商"读出错就 while(res){ sd_init(); 重试 }"的【无限重试】: 该循环在卡
 *     掉线时会永久卡死主循环 → 改为【有限重试 3 次】后返回 RES_ERROR, 由上层计丢包
 *   - 只保留 SD 卡一个卷 (FF_VOLUMES=1)
 */
#include "diskio.h"
#include "sdio_sdcard.h"

#define DEV_SD              0
#define SD_RETRY_CNT        3       /* 单次传输失败后的重试次数 (有限, 不卡死) */

DSTATUS disk_status(BYTE pdrv)
{
    return (pdrv == DEV_SD) ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != DEV_SD) {
        return STA_NOINIT;
    }
    return (sd_init() == 0) ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != DEV_SD || buff == NULL || count == 0) {
        return RES_PARERR;
    }
    for (uint8_t i = 0; i < SD_RETRY_CNT; i++) {
        if (sd_read_disk(buff, sector, count) == 0) {
            return RES_OK;
        }
    }
    return RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != DEV_SD || buff == NULL || count == 0) {
        return RES_PARERR;
    }
    for (uint8_t i = 0; i < SD_RETRY_CNT; i++) {
        if (sd_write_disk((uint8_t *)buff, sector, count) == 0) {
            return RES_OK;
        }
    }
    return RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != DEV_SD) {
        return RES_PARERR;
    }

    switch (cmd) {
        case CTRL_SYNC:
            return (get_sd_card_state() == SD_TRANSFER_OK) ? RES_OK : RES_ERROR;

        case GET_SECTOR_COUNT:
            *(DWORD *)buff = g_sd_card_info_handle.LogBlockNbr;
            return RES_OK;

        case GET_SECTOR_SIZE:               /* FF_MAX_SS = 512 */
            *(WORD *)buff = 512;
            return RES_OK;

        case GET_BLOCK_SIZE:                /* 擦除块大小(扇区数); 1 = 不关心 */
            *(WORD *)buff = 1;
            return RES_OK;

        default:
            return RES_PARERR;
    }
}

/* FatFS 建/改文件时间戳。本工程无 RTC 电池, 用固定值 (FF_FS_NORTC 亦可置 1 免调用) */
DWORD get_fattime(void)
{
    return ((DWORD)(2026 - 1980) << 25) | ((DWORD)3 << 21) | ((DWORD)1 << 16);
}
