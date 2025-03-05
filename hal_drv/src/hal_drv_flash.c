#include "hal_drv_flash.h"
#include "ql_embed_nor_flash.h"




int hal_drv_flash_write(uint32_t addr, void *data, size_t len)
{
    return ql_embed_nor_flash_write(addr, data, len);
}


int hal_drv_flash_read(uint32_t addr, void *data, size_t len)
{
    return ql_embed_nor_flash_read(addr, data, len);
}

int hal_drv_flash_erase(uint32_t addr, size_t len)
{
    return ql_embed_nor_flash_erase(addr, len);
}

