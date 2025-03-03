#include "app_system.h"
#include "ql_http_client.h"
#include "ql_fs.h"
#include "http_upgrade_ota.h"
#include "ql_api_dev.h"
#include "hal_drv_net.h"
#include "ql_api_fota.h"
#define DBG_TAG         "http_upgrade"

#ifdef HTTP_UPGRADE_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

#define TRY_DOWN_TIMES		10
#define QL_FOTA_PACK_NAME_MAX_LEN         (172)
#define HTTP_HEAD_RANGE_LENGTH_MAX  50
#define WRITE_TO_FILESIZE	(1024*5)	
def_rtos_sem_t ota_http_sem;

typedef enum 
{
    OTA_HTTP_DOWN_INIT,
    OTA_HTTP_DOWN_DOWNING,
    OTA_HTTP_DOWN_INTR,
    OTA_HTTP_DOWN_DOWNED,
    OTA_HTTP_DOWN_NOSPACE,
} e_ota_down_stage;

struct http_upgrade_info_stu http_upgrade_info;
struct fota_http_progress_stu {
    bool is_show;
    uint32_t total_size;
    uint32_t dload_size;
    uint32_t file_size; 
};
struct fota_http_client_stu {
    http_client_t http_cli;	
    bool b_is_http_range;
    int profile_idx;					//cid
    uint8_t sim_id;						//simid
    char fota_packname[QL_FOTA_PACK_NAME_MAX_LEN];
    struct fota_http_progress_stu http_progress;
    uint8_t http_res;
    e_ota_down_stage e_stage;
    QFILE	fd;	
    int i_save_size;
    uint32_t last_precent;
    bool b_is_have_space;
    int chunk_encode;
};

static int ota_http_get_filesize(char *filename)
{
    int file_size = 0;
    QFILE fd = ql_fopen(filename, "rb");
    if(fd < 0) {
        LOG_E("open file name:[%s] ret = %d", filename, fd);
    }
    file_size = ql_fsize(fd);
    ql_fclose(fd);
    return file_size;
}

int ota_http_init(struct fota_http_client_stu *fota_http_cli_p)
{
    QFILE fd = -1;
    memset(fota_http_cli_p, 0x00, sizeof(struct fota_http_client_stu));

    fota_http_cli_p->http_cli = 0;
    fota_http_cli_p->profile_idx = http_upgrade_info.profile_idx;
    fota_http_cli_p->sim_id = 0;
    fota_http_cli_p->e_stage = OTA_HTTP_DOWN_INIT;
    fota_http_cli_p->i_save_size = 0;
    
    fota_http_cli_p->b_is_have_space = true;
    fota_http_cli_p->http_progress.is_show = true;
    fota_http_cli_p->last_precent = 0;
    fota_http_cli_p->chunk_encode = 0;
    switch(sys_param_set.farme_type)
    {
        case IOT_FIRMWARE_TYPE:
        case BLUE_FIRMWARE_TYPE:
        case ECU_FIRMWARE_TYPE:
        case BMS1_FIRMWARE_TYPE:
        case BMS2_FIRMWARE_TYPE:
        case HMI_FIRMWARE_TYPE:
        case LOCK_FIRMWARE_TYPE:
        case MCU_FIRMWARE_TYPE:
            memcpy(fota_http_cli_p->fota_packname, "UFS:fota.pack", strlen("UFS:fota.pack"));
        break;
        case VOICE_PACK_TYPE1:
            memcpy(fota_http_cli_p->fota_packname, UNLOCK_VOICE_FILE, strlen(UNLOCK_VOICE_FILE));
        break;
        case VOICE_PACK_TYPE2:
            memcpy(fota_http_cli_p->fota_packname, LOCK_VOICE_FILE, strlen(LOCK_VOICE_FILE));
        break;
        case VOICE_PACK_TYPE3:
            memcpy(fota_http_cli_p->fota_packname, ALARM_VOICE_FILE, strlen(ALARM_VOICE_FILE));
        break;
        case VOICE_PACK_TYPE4:
            memcpy(fota_http_cli_p->fota_packname, LOOK_CAR_VOICE_FILE, strlen(LOOK_CAR_VOICE_FILE)); 
        break;
        case VOICE_PACK_TYPE5:
            memcpy(fota_http_cli_p->fota_packname, DEFINE_VOICE_FILE, strlen(DEFINE_VOICE_FILE)); 
        break;
    }
    
    fd = ql_fopen(fota_http_cli_p->fota_packname, "wb+");
    if(fd < 0) {
        LOG_E("init file name:[%s] ret:%d", fota_http_cli_p->fota_packname, fd);
        return -1;
    }
    ql_fclose(fd);
    LOG_I("init file name:[%s]", fota_http_cli_p->fota_packname);
    LOG_I("init file size:[%d]", ota_http_get_filesize(fota_http_cli_p->fota_packname));
    LOG_I("init file stage:[%d]", fota_http_cli_p->e_stage);
    return 0;
}
static void fota_http_info_cfg(struct fota_http_client_stu* fota_http_cli_p)
{
    if(fota_http_cli_p == NULL) {
        LOG_E("fota_http_cli_p is null");
        return;
    }
    LOG_I("init file name:[%s]",fota_http_cli_p->fota_packname);
	LOG_I("init file stage:[%d]",fota_http_cli_p->e_stage);
	LOG_I("init file download:[%d]",fota_http_cli_p->http_progress.dload_size);
	LOG_I("init file file_size:[%d]",fota_http_cli_p->http_progress.file_size);
	LOG_I("init file real file_size:[%d]",ota_http_get_filesize(fota_http_cli_p->fota_packname));
	LOG_I("init file is_show:[%d]",fota_http_cli_p->http_progress.is_show);
	LOG_I("init file last_percent:[%d]",fota_http_cli_p->last_precent);
	LOG_I("init file space:[%d]",fota_http_cli_p->b_is_have_space);
}

static QFILE ota_http_get_fd(struct fota_http_client_stu *fota_http_cli_p)
{

    if(fota_http_cli_p->e_stage == OTA_HTTP_DOWN_INIT || fota_http_cli_p->e_stage == OTA_HTTP_DOWN_DOWNED\
    ||fota_http_cli_p->e_stage == OTA_HTTP_DOWN_NOSPACE)
    {
        fota_http_cli_p->fd = ql_fopen(fota_http_cli_p->fota_packname, "wb+");
		fota_http_cli_p->http_progress.file_size = 0;
		fota_http_cli_p->http_progress.dload_size = 0;
		fota_http_cli_p->http_progress.total_size = 0;
        fota_http_cli_p->e_stage = OTA_HTTP_DOWN_INIT;
        fota_http_cli_p->i_save_size = 0;
        LOG_I("over write file [%s]", fota_http_cli_p->fota_packname);
    } else {
        fota_http_cli_p->fd = ql_fopen(fota_http_cli_p->fota_packname, "ab+");
        LOG_I("add write file [%s]", fota_http_cli_p->fota_packname);
    }
    if(fota_http_cli_p->fd < 0) {
        LOG_E("ql_open failed");
    }
    return fota_http_cli_p->fd;
}

static void ota_http_close_fd(struct fota_http_client_stu *fota_http_cli_p)
{
    if(fota_http_cli_p->fd > 0) {
        ql_fclose(fota_http_cli_p->fd);
        fota_http_cli_p->fd = -1;
    }
}
int ota_dload_file_clran(struct fota_http_client_stu * fota_http_cli_p)
{
    if(fota_http_cli_p->fd < 0) {
        LOG_E("clran write file [%s] %d", fota_http_cli_p->fota_packname, fota_http_cli_p->fd);
        fota_http_cli_p->fd = -1;
    } else {
        ql_fclose(fota_http_cli_p->fd);
        def_rtos_task_sleep_ms(10);
    }
    fota_http_cli_p->fd = ql_fopen(fota_http_cli_p->fota_packname, "wb+");
    if(fota_http_cli_p->fd < 0) {
        LOG_E("clran open write file [%s] failed %d", fota_http_cli_p->fota_packname, fota_http_cli_p->fd);
        return -1;
    }
    fota_http_cli_p->http_progress.file_size = 0;
    fota_http_cli_p->http_progress.dload_size = 0;
    fota_http_cli_p->http_progress.total_size = 0;
    fota_http_cli_p->e_stage = OTA_HTTP_DOWN_DOWNING;
    fota_http_cli_p->i_save_size = 0;
    LOG_E("clran write file [%s] open fd %d", fota_http_cli_p->fota_packname,fota_http_cli_p->fd);
    return 0;
}

static void ota_http_event_cb(http_client_t *client, int event, int event_code, void *argv)
{
    if(argv == NULL) {
        LOG_E("fota_http_event_cb argv is null");
        return;
    }
    struct fota_http_client_stu * fota_http_cli_p = (struct fota_http_client_stu*)argv;
    if(*client != fota_http_cli_p->http_cli) 
        return;
    switch(event)
    {
        case HTTP_EVENT_SESSION_ESTABLISH: {
            if(event_code != HTTP_SUCCESS) {
                LOG_E("http session create failed!!!!!");
                if(fota_http_cli_p->e_stage != OTA_HTTP_DOWN_NOSPACE && fota_http_cli_p->e_stage != OTA_HTTP_DOWN_DOWNED)
                {
                    fota_http_cli_p->e_stage = OTA_HTTP_DOWN_INTR;
                }
                def_rtos_smaphore_release(ota_http_sem);
                fota_http_cli_p->http_res = 0;
            }
        }
        break;
        case HTTP_EVENT_RESPONE_STATE_LINE:
            if(event_code == HTTP_SUCCESS) {
                int resp_code = 0;
	            int content_length = 0;
	            int chunk_encode = 0;
                int accept_ranges = 0;
	            char *location = NULL;
                ql_httpc_getinfo(client, HTTP_INFO_RESPONSE_CODE, &resp_code);
	            ql_httpc_getinfo(client, HTTP_INFO_CHUNK_ENCODE, &chunk_encode);
                LOG_I("response code:%d chunk_encode %d", resp_code, chunk_encode);
                fota_http_cli_p->e_stage = OTA_HTTP_DOWN_DOWNING;
                if(resp_code == 200 || resp_code == 206) {
                    if(chunk_encode == 0)
                    {
                        ql_httpc_getinfo(client, HTTP_INFO_ACCEPT_RANGES, &accept_ranges);
                        ql_httpc_getinfo(client, HTTP_INFO_CONTENT_LEN, &content_length);
                        if(accept_ranges == 1 &&  fota_http_cli_p->b_is_http_range == true)
                        {
                            fota_http_cli_p->http_progress.total_size  += content_length;
                        }
                        else 
                        {
                            if(ota_dload_file_clran(fota_http_cli_p) == 0)
                            {
                                fota_http_cli_p->http_progress.total_size = content_length;
                            }
                            else
                            {
				                fota_http_cli_p->e_stage = OTA_HTTP_DOWN_DOWNED;
                            }
                        }
                        LOG_I("content_length:[%d] totalsize=[%d]", content_length, fota_http_cli_p->http_progress.total_size);
                    } else if(1 == chunk_encode) {
                    LOG_I("http chunk encode!");
                    fota_http_cli_p->chunk_encode = 1;
                    }
                } else {
                    fota_http_cli_p->e_stage = OTA_HTTP_DOWN_DOWNED;
                }
                if(resp_code == 416) {
                    fota_http_cli_p->e_stage = OTA_HTTP_DOWN_DOWNED;
                }
                if(resp_code >= 300 && resp_code < 400) {
                    fota_http_cli_p->e_stage = OTA_HTTP_DOWN_DOWNED;
                    ql_httpc_getinfo(client, HTTP_INFO_LOCATION, &location);
	                LOG_I("redirect location:%s", location);
	                free(location);
                }
            } 
        break;
        case HTTP_EVENT_SESSION_DISCONNECT:
            if(event_code == HTTP_SUCCESS)
            {
                fota_http_cli_p->e_stage = OTA_HTTP_DOWN_DOWNED;
                LOG_I("==http transfer end!!!");
            } else {
                if(fota_http_cli_p->e_stage != OTA_HTTP_DOWN_NOSPACE && fota_http_cli_p->e_stage != OTA_HTTP_DOWN_DOWNED)
                {
                    fota_http_cli_p->e_stage = OTA_HTTP_DOWN_INTR;
                }
                LOG_I("===http transfer occur execption!!!");
            }
            def_rtos_smaphore_release(ota_http_sem);
            fota_http_cli_p->http_res = 1;
        break;
    }
}
static int ota_http_write_file(struct fota_http_client_stu* fota_cli_p ,char *data, int size,  QFILE fd)
{
    int ret = -1;
	uint temp=0;
    if(fota_cli_p->fd != fd)
    {
        LOG_E("file fd error");
        ota_http_close_fd(fota_cli_p);
        return 0;
    }
	//写文件前休息1ms,以防永久阻塞
	def_rtos_task_sleep_ms(1);
    LOG_I("write [%d] size",size);
    ret = ql_fwrite(data, size,1,fd);
	LOG_I("write ret=[%d]",ret);
    if (ret > 0)
    {
        fota_cli_p->http_progress.dload_size += (uint)ret;
		fota_cli_p->http_progress.file_size = ql_fsize(fd);
		if ( fota_cli_p->http_progress.is_show == true )
		{
            if (1 != fota_cli_p->chunk_encode)
            {
                //计算进度，如果开启进度显示，那么会计算本次进度和上次进度是否相同，进度不同才会展示进度
                temp = 100UL*fota_cli_p->http_progress.dload_size/fota_cli_p->http_progress.total_size;
                if ( fota_cli_p->last_precent != temp || temp == 100  )
                {
                    fota_cli_p->last_precent = temp;
                    LOG_I("dload progress:===[%u%%]===total size[%d] file_size[%d] dload size[%d]",temp,fota_cli_p->http_progress.total_size,ql_fsize(fd),fota_cli_p->http_progress.dload_size );
                }
            }
            else
            {
                LOG_I("dload progress:=== file_size[%d] dload size[%d] ===", ql_fsize(fd), fota_cli_p->http_progress.dload_size);
            }
		}

		//保存文件，每一次满5k,保存一次写入的文件
		if ( (fota_cli_p->i_save_size <= fota_cli_p->http_progress.dload_size)
          || ( (1 != fota_cli_p->chunk_encode) && (fota_cli_p->i_save_size >= fota_cli_p->http_progress.total_size) ) )
		{
			//满WRITE_TO_FILESIZE个字节保存一次
			if ( (1 != fota_cli_p->chunk_encode) && (fota_cli_p->i_save_size >= fota_cli_p->http_progress.total_size) )
			{
				fota_cli_p->i_save_size = fota_cli_p->http_progress.total_size;
			}
			else
			{
				fota_cli_p->i_save_size=fota_cli_p->http_progress.dload_size+WRITE_TO_FILESIZE;
			}
		}
		if ( (1 != fota_cli_p->chunk_encode) && (fota_cli_p->http_progress.dload_size >= fota_cli_p->http_progress.total_size) )
		{
			fota_cli_p->e_stage = OTA_HTTP_DOWN_DOWNED;
		}
		if ( ret != size )
		{
			//关闭固件升级包的文件描述符
			ota_http_close_fd(fota_cli_p);
		}
    }
    else
    {
        LOG_E("error: ret:%d",ret);
		//关闭固件升级包的文件描述符
	    ota_http_close_fd(fota_cli_p);
    }
	return ret;    
}
static int ota_http_write_response_data(http_client_t *client, void *argv, char *data, int size, unsigned char end)
{
    int ret = -1;
    int write_size = size;
    char *p_write_data = data;
    int i_deal_size = WRITE_TO_FILESIZE;
    int64 file_free_size=0;
    if(argv == NULL) {
        LOG_E("fota_http_write_response_data argv is invalied NULL");
        return -2;
    }
    struct fota_http_client_stu * fota_cli_p = (struct fota_http_client_stu*)argv;

    if((fota_cli_p->e_stage == OTA_HTTP_DOWN_DOWNED) || (fota_cli_p->chunk_encode == 1 && end == 1)){
        fota_cli_p->e_stage = OTA_HTTP_DOWN_DOWNED;
        LOG_I("%s go on dload file finished!", fota_cli_p->fota_packname);
        ota_http_close_fd(fota_cli_p);
        return 0;
    }
    file_free_size = ql_fs_free_size(fota_cli_p->fota_packname);
    if ( (1 != fota_cli_p->chunk_encode && file_free_size < (fota_cli_p->http_progress.total_size - fota_cli_p->http_progress.dload_size))
      || (1 == fota_cli_p->chunk_encode &&  file_free_size < size) )
    {
        if(1 != fota_cli_p->chunk_encode) {
            LOG_I("free_space[%d] total_size [%d] dload_size[%d]", file_free_size, fota_cli_p->http_progress.total_size\
		        ,fota_cli_p->http_progress.dload_size);
        } else{
            LOG_I("free_space[%d] dload_size[%d]",file_free_size,fota_cli_p->http_progress.dload_size);
        }
        fota_cli_p->e_stage = OTA_HTTP_DOWN_NOSPACE;
        fota_cli_p->b_is_have_space = false;
        LOG_E("file free_size not enough");
        ota_http_close_fd(fota_cli_p);
        return 0;
    }

    if(size <= 0) {
        LOG_E("write 0 size to file [%s]",fota_cli_p->fota_packname);
        ota_http_close_fd(fota_cli_p);
        return -1;
    }

    do {
        if(write_size < i_deal_size)
        {
            i_deal_size = write_size;
        }
        ret = ota_http_write_file(fota_cli_p ,p_write_data, i_deal_size,  fota_cli_p->fd);
        if(ret < 0) {
            LOG_E("write file error");
            return size - write_size;
        }
        write_size -= ret;
        p_write_data += ret;
    }while(write_size > 0);
    return size - write_size;
}


static int ota_http_evn_request(struct fota_http_client_stu *fota_http_cli_p)
{
    char dload_range[HTTP_HEAD_RANGE_LENGTH_MAX] = {0};
    http_method_e e_http_method;
    if(ota_http_get_fd(fota_http_cli_p) < 0) {
        LOG_E("range_request http data done ,file_size[%d]",fota_http_cli_p->http_progress.file_size);
        return -1;
    }

    if(ql_httpc_new(&(fota_http_cli_p->http_cli), ota_http_event_cb, fota_http_cli_p) != HTTP_SUCCESS){
        LOG_E("http create failed");
        ql_httpc_release(&(fota_http_cli_p->http_cli));
        ota_http_close_fd(fota_http_cli_p);
        return -2;
    }
    
    e_http_method = HTTP_METHOD_GET;
    ql_httpc_setopt(&(fota_http_cli_p->http_cli), HTTP_CLIENT_OPT_METHOD,e_http_method);

    if(fota_http_cli_p->b_is_http_range == true) {
        fota_http_cli_p->http_progress.dload_size = fota_http_cli_p->http_progress.file_size;
		sprintf(dload_range, "Range: bytes=%ld-",fota_http_cli_p->http_progress.file_size); 
		ql_httpc_setopt(&(fota_http_cli_p->http_cli), HTTP_CLIENT_OPT_REQUEST_HEADER,dload_range);
		LOG_I("Get http %s",dload_range);
    } else {

    }
    LOG_I("URL:%s", http_upgrade_info.url);
    	//设置url下载地址
	ql_httpc_setopt(&(fota_http_cli_p->http_cli), HTTP_CLIENT_OPT_URL, http_upgrade_info.url);
	//设置sim_id
	ql_httpc_setopt(&(fota_http_cli_p->http_cli), HTTP_CLIENT_OPT_SIM_ID, fota_http_cli_p->sim_id);
	//设置cid
	ql_httpc_setopt(&(fota_http_cli_p->http_cli), HTTP_CLIENT_OPT_PDPCID, fota_http_cli_p->profile_idx);
	//接收报体中的文件内容
    ql_httpc_setopt(&(fota_http_cli_p->http_cli), HTTP_CLIENT_OPT_WRITE_FUNC, ota_http_write_response_data);
	//设置fota_http_write_response_data 第二参数为fota_http_cli
    ql_httpc_setopt(&(fota_http_cli_p->http_cli), HTTP_CLIENT_OPT_WRITE_DATA, fota_http_cli_p);

    if (ql_httpc_perform(&fota_http_cli_p->http_cli) == HTTP_SUCCESS)
	{
		 //阻塞等待信号量
		if (def_rtos_semaphore_wait(ota_http_sem, QL_WAIT_FOREVER) != QL_OSI_SUCCESS )
		 {
		 	//获取信号量失败
		 	ql_httpc_release(&(fota_http_cli_p->http_cli));
		 	ota_http_close_fd(fota_http_cli_p);
		 	return -1;
		 }
         if(fota_http_cli_p->http_res == 1){
            LOG_I("fota http dload size %d=====End,\n",fota_http_cli_p->http_progress.dload_size);
		    ql_httpc_release(&(fota_http_cli_p->http_cli));
		    ota_http_close_fd(fota_http_cli_p);
		    return 0;
         } 
		 goto exit;
	}
exit: ql_httpc_release(&(fota_http_cli_p->http_cli));
	 ota_http_close_fd(fota_http_cli_p);
	 return -3;

}

static int ota_http_download_pacfile(struct fota_http_client_stu *fota_http_cli_p)
{
//    ql_errcode_fota_e ret;
    fota_http_info_cfg(fota_http_cli_p);
    if(ota_http_evn_request(fota_http_cli_p) != 0)
    {
        int file_size = ota_http_get_filesize(fota_http_cli_p->fota_packname);
        LOG_E("failed [%s] size[%d]",fota_http_cli_p->fota_packname,file_size);
        return -1;
    }
    fota_http_info_cfg(fota_http_cli_p);
/*
    if(fota_http_cli_p->e_stage == OTA_HTTP_DOWN_DOWNED)
    {
        switch (http_upgrade_info.farme_type){
            case IOT_FIRMWARE_TYPE:
             ret = ql_fota_image_verify(fota_http_cli_p->fota_packname);
		    if ( ret != RTOS_SUCEESS )
		    {
			    //下载完成校验不成功删除文件
			    ql_remove(fota_http_cli_p->fota_packname);
			    LOG_E("[%s]package is invalid", fota_http_cli_p->fota_packname);
			    return -3;
		    } else {
			//校验成功
			    LOG_I("download is sucess ,system will reset power!");
                def_rtos_task_sleep_s(5);
	            sys_reset();
		    }
            break;
        }
    }*/
    return 0;
}

int app_iot_ota_jump()
{
    ql_errcode_fota_e ret;
    ret = ql_fota_image_verify(sys_info.fota_packname);
    if(ret != RTOS_SUCEESS) {
        ql_remove(sys_info.fota_packname);
        return -3;
    } else {
        net_engwe_fota_state_push(FOTA_SATRT_UPDATE);
        LOG_I("download is sucess ,system will reset power!");
        def_rtos_task_sleep_s(5);
	    sys_reset();
        return 0;
    }
}
void app_http_ota_init()
{
    def_rtos_semaphore_create(&ota_http_sem, 0);
    def_rtos_semaphore_create(&http_upgrade_info.http_ota_sem, 0);
    memset(http_upgrade_info.url, 0, sizeof(http_upgrade_info.url)); 
    http_upgrade_info.ota_stage = HTTP_OTA_WAIT;
    http_upgrade_info.profile_idx = 2;
}

void http_upgrade_start()
{
    def_rtos_smaphore_release(http_upgrade_info.http_ota_sem);

}

void http_upgrade_stop()
{
    http_upgrade_info.stop_flag = 1;
    def_rtos_smaphore_release(ota_http_sem);
}

uint32_t fw_check_sum()
{
    uint32_t check_sum = 0;
    uint8_t data;
    uint32_t ota_size, offset = 0;
    ota_size = flash_partition_size(DEV_APP_ADR);
    while(ota_size > 0){
        check_sum += flash_partition_read(DEV_APP_ADR, &data, 1, offset);
        offset++;
        ota_size--;
    }
    return check_sum;
}

void http_ota_fw_update()
{
    switch(sys_param_set.farme_type){
        case IOT_FIRMWARE_TYPE:
            app_iot_ota_jump();
        break;
        case BLUE_FIRMWARE_TYPE:
            net_engwe_fota_state_push(FOTA_SATRT_UPDATE);
            if(ble_ota_task() == OK){
                net_engwe_fota_state_push(FOTA_UPDATE_SUCCESS);
            } else {
                net_engwe_fota_state_push(FOTA_UPDATE_FAIL);
            }
        break;
        case MCU_FIRMWARE_TYPE:
            net_engwe_fota_state_push(FOTA_SATRT_UPDATE);
            if(mcu_ota_task() == OK) {
                net_engwe_fota_state_push(FOTA_UPDATE_SUCCESS);
            } else {
                net_engwe_fota_state_push(FOTA_UPDATE_FAIL);
            }
        break;
        case VOICE_PACK_TYPE1:
        case VOICE_PACK_TYPE2:
        case VOICE_PACK_TYPE3:
        case VOICE_PACK_TYPE4:
        case VOICE_PACK_TYPE5:
            net_engwe_fota_state_push(FOTA_UPDATE_SUCCESS);
        break;
        case ECU_FIRMWARE_TYPE:
            net_engwe_fota_state_push(FOTA_SATRT_UPDATE);
            if(can_ota_task(CONTROL_ADR) == OK) {
                net_engwe_fota_state_push(FOTA_UPDATE_SUCCESS);
            }
        break;
        case BMS1_FIRMWARE_TYPE:
            net_engwe_fota_state_push(FOTA_SATRT_UPDATE);
            if(can_ota_task(BMS_ADR) == OK) {
                net_engwe_fota_state_push(FOTA_UPDATE_SUCCESS);
            }
        break;
        case BMS2_FIRMWARE_TYPE:
            net_engwe_fota_state_push(FOTA_SATRT_UPDATE);
            if(can_ota_task(SECOND_BMS_ADR) == OK) {
                net_engwe_fota_state_push(FOTA_UPDATE_SUCCESS);
            }
        break;
        case HMI_FIRMWARE_TYPE:
            net_engwe_fota_state_push(FOTA_SATRT_UPDATE);
            if(can_ota_task(HMI_ADR) == OK) {
                net_engwe_fota_state_push(FOTA_UPDATE_SUCCESS);
            }
        break;
        case LOCK_FIRMWARE_TYPE:
            net_engwe_fota_state_push(FOTA_SATRT_UPDATE);
            if(can_ota_task(LOCK_ADR) == OK) {
                net_engwe_fota_state_push(FOTA_UPDATE_SUCCESS);
            }
        break;
    }
}
int http_pdp_active()
{
    static uint8_t step = 0;
    static uint64_t pdp_time_t;
    switch(step) {
        case 0:
            if(hal_drv_get_net_register_sta() != 1){
                LOG_E("net register is fail");
                return -1;
            } else if(hal_drv_get_data_call_res(http_upgrade_info.profile_idx, NULL) != 1) {
                hal_drv_set_data_call_asyn_mode(http_upgrade_info.profile_idx, 1);
                hal_drv_start_data_call(http_upgrade_info.profile_idx, sys_config.apn); 
                step = 1;
                pdp_time_t = def_rtos_get_system_tick();
            } else {
                return 0;
            }
        break;
        case 1:
            if(hal_drv_get_data_call_res(http_upgrade_info.profile_idx, NULL)) {
                step = 0;
                return 0;
            } else if((def_rtos_get_system_tick() - pdp_time_t)/1000 > 5*60){
                step = 0;
                return -1;
            }
            break;
    }
    return 1;
}
void app_http_ota_thread(void *param)
{
 //   int64_t http_start_time_t;
    def_rtosStaus res;
    int pdp_res;
//    uint8_t down_times;
    uint8_t temp = 0;
    struct fota_http_client_stu  fota_http_cli_p;
    char version_buf[256] = {0};
    ql_dev_get_firmware_version(version_buf, sizeof(version_buf));
    LOG_I("current version:  %s", version_buf);
    def_rtos_task_sleep_ms(10000);
    while(1) {  
        switch(http_upgrade_info.ota_stage)
        {
            case HTTP_OTA_WAIT:
                sys_info.ota_flag = 0;
                res = def_rtos_semaphore_wait(http_upgrade_info.http_ota_sem, RTOS_WAIT_FOREVER);
                if(res != RTOS_SUCEESS) {
                    continue;
                } 
                http_upgrade_info.ota_stage = HTTP_OTA_CHECK;
                sys_info.ota_flag = 1;
                http_upgrade_info.stop_flag = 0;
                break;
            case HTTP_OTA_CHECK:
                if(sys_info.power_36v == 0) {
                    net_engwe_fota_state_push(FOTA_NOT_BAT);
                    http_upgrade_info.ota_stage = HTTP_OTA_WAIT;
                } else {
                    http_upgrade_info.ota_stage = HTTP_PDP_ACTIVE;
                }
            break;
            case HTTP_PDP_ACTIVE:
                pdp_res = http_pdp_active();
                if(pdp_res == 0) {
                    http_upgrade_info.ota_stage = HTTP_OTA_INIT;
                } else if(pdp_res == -1) {
                    http_upgrade_info.ota_stage = HTTP_OTA_WAIT;
                }
                def_rtos_task_sleep_ms(100);
            break;
            case HTTP_OTA_INIT:
                ota_http_init(&fota_http_cli_p);
                http_upgrade_info.ota_stage = HTTP_OTA_DOWN;
            break;
            case HTTP_OTA_DOWN:
                net_engwe_fota_state_push(FOTA_START_DOWN);   
                if(ota_http_download_pacfile(&fota_http_cli_p) == 0) {
                    http_upgrade_info.ota_stage = HTTP_OTA_SUM_CHECK;
                    hal_drv_stop_data_call(http_upgrade_info.profile_idx);
                } else {
                    hal_drv_stop_data_call(http_upgrade_info.profile_idx);   //取消拨号
                    if(http_upgrade_info.stop_flag == 1) {
                        
                    } else {
                        net_engwe_fota_state_push(FOTA_DOWN_FAIL);
                    }
                    http_upgrade_info.ota_stage = HTTP_OTA_WAIT;
                }
            break;
            case HTTP_OTA_SUM_CHECK:
                memset(sys_info.fota_packname, 0, sizeof(sys_info.fota_packname));
                memcpy(sys_info.fota_packname, fota_http_cli_p.fota_packname, strlen(fota_http_cli_p.fota_packname));
                if(http_upgrade_info.crc_sum == fw_check_sum()){
                    net_engwe_fota_state_push(FOTA_DOWN_SUCCESS);
                    http_upgrade_info.ota_stage = HTTP_OTA_UPDATA;
                } else {
                    net_engwe_fota_state_push(FOTA_DOWN_FAIL);
                    http_upgrade_info.ota_stage = HTTP_OTA_WAIT;
                }
            break;
            case HTTP_OTA_UPDATA:
                http_ota_fw_update();
                http_upgrade_info.ota_stage = HTTP_OTA_WAIT;
            break;
        }
            // for(;;){
            //     LOG_I("enter can_ota_task");
            //     if(can_ota_task(HMI_ADR) == OK){
            //         sys_param_set.ota_cnt++;
            //         sys_param_save(SYS_SET_ADR);
            //         LOG_I("ota_cnt:%d", sys_param_set.ota_cnt);
            //     }
            //     def_rtos_task_sleep_s(10);
            // }
        if(temp == 0) {
            temp = 1;
          // LOG_I("enter ble_ota_task");
        //   ble_ota_task();
        //    can_ota_task(HMI_ADR);
    //        mcu_ota_task();
        }    

    //    LOG_I("IS RUN");
   /*     res = def_rtos_semaphore_wait(http_upgrade_info.http_ota_sem, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        } 
        http_upgrade_info.download_fail_cent = 0;
        http_upgrade_info.download_start_byte = 0;
        http_start_time_t = def_rtos_get_system_tick();
        if(sys_info.power_36v == 0 &&  sys_info.bat_soc < 80) {
            http_upgrade_info.ota_sta = IOT_BAT_LOW;
            NET_CMD_MARK(NET_CMD_HTTP_UPGRADE_STATE_U6);
            continue;
        }
    
        http_upgrade_info.ota_sta = IOT_RECV_OTA_REQ;
        NET_CMD_MARK(NET_CMD_HTTP_UPGRADE_STATE_U6);
        down_times = 10;
        def_rtos_task_sleep_s(5);
        sys_info.ota_flag = 1;
        net_socket_close();
        def_rtos_task_sleep_s(2);
        ota_http_init(&fota_http_cli_p);
        while(down_times--) {
            if(def_rtos_get_system_tick() - http_start_time_t > http_upgrade_info.timeout * 60*1000) {
                http_upgrade_info.ota_sta = IOT_DOWNLOAD_FAIL;
                 NET_CMD_MARK(NET_CMD_HTTP_UPGRADE_STATE_U6);
                 break;
            }
            if(ota_http_download_pacfile(&fota_http_cli_p) == 0)
            {
                LOG_I("http upgrade ota is ok");
                break;
            }
            if(fota_http_cli_p.b_is_have_space != true)
            {
                ql_remove(fota_http_cli_p.fota_packname);
			    LOG_E("have no space");
			    break;
            }
            def_rtos_task_sleep_s(40);
        }
        sys_info.ota_flag = 0;  */
    }
    def_rtos_task_delete(NULL);
}
