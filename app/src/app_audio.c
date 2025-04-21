#include    "app_system.h"
#include    "ql_audio.h"

#define DBG_TAG         "app_audio"

#ifdef APP_AUDIO_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

def_rtos_queue_t audio_que_t;


struct audio_con_stu
{
    uint8_t play_cnt;
    uint32_t play_interval;
    char file_name[56];
};



struct audio_con_stu audio_con_table[] = {
    {1,     0,      LOCK_VOICE_FILE},
    {1,     0,      UNLOCK_VOICE_FILE},
    {1,    0,   ALARM_VOICE_FILE},
    {5,    1000,   LOOK_CAR_VOICE_FILE},
    {0Xff,     1000,   ENTER_PENALTY_AREA_VOICE_FILE},
};

uint8_t play_flag;

void voice_play_mark(VOICE_TYPE type)
{
    AudioStatus_e audio_state;
    if(sys_info.audio_init == 0) return; 
    play_flag = 1;
    audio_state = ql_aud_get_play_state();
    if(audio_state != QL_AUDIO_STATUS_IDLE) {
        ql_aud_player_stop();
    }
    def_rtos_queue_release(audio_que_t, sizeof(uint8_t), (uint8_t *)&type,  RTOS_WAIT_FOREVER); 
}


void voice_play_off()
{
    AudioStatus_e audio_state;
    if(sys_info.audio_init == 0) return; 
    audio_state = ql_aud_get_play_state();
    if(audio_state != QL_AUDIO_STATUS_IDLE) {
        ql_aud_player_stop();
    }
    play_flag = 1;
}

void audio_play_test()
{
    def_rtosStaus res;
    res = ql_aud_play_file_start(LOCK_VOICE_FILE, QL_AUDIO_PLAY_TYPE_LOCAL, NULL);
    if(res) {
        LOG_E("ql_aud_play_file_start fail,res:%d", res);
    }
    res = ql_aud_wait_play_finish(QL_WAIT_FOREVER);
    if(res) {
        LOG_E("ql_aud_wait_play_finish,res:%d", res);
    }
    def_rtos_task_sleep_ms(1000);
    res = ql_aud_play_file_start(UNLOCK_VOICE_FILE, QL_AUDIO_PLAY_TYPE_LOCAL, NULL);
    if(res) {
        LOG_E("ql_aud_play_file_start fail,res:%d", res);
    }
    res = ql_aud_wait_play_finish(QL_WAIT_FOREVER);
    if(res) {
        LOG_E("ql_aud_wait_play_finish,res:%d", res);
    }
    def_rtos_task_sleep_ms(1000);
    res = ql_aud_play_file_start(ALARM_VOICE_FILE, QL_AUDIO_PLAY_TYPE_LOCAL, NULL);
    if(res) {
        LOG_E("ql_aud_play_file_start fail,res:%d", res);
    }
    res = ql_aud_wait_play_finish(QL_WAIT_FOREVER);
    if(res) {
        LOG_E("ql_aud_wait_play_finish,res:%d", res);
    }
    def_rtos_task_sleep_ms(1000);
    res = ql_aud_play_file_start(LOOK_CAR_VOICE_FILE, QL_AUDIO_PLAY_TYPE_LOCAL, NULL);
    if(res) {
        LOG_E("ql_aud_play_file_start fail,res:%d", res);
    }
    res = ql_aud_wait_play_finish(QL_WAIT_FOREVER);
    if(res) {
        LOG_E("ql_aud_wait_play_finish,res:%d", res);
    }
    def_rtos_task_sleep_ms(1000);
}


void app_audio_init()
{
	int err = 0;

	ql_set_audio_path_receiver();
    //配置本地播放模式下, 喇叭输出的1级音量对应的dac增益为 -13.5db, 算法增益为0db, 且实时生效
	err = ql_aud_set_icvolume_level_gain(QL_AUDIO_PLAY_TYPE_LOCAL,
								   QL_OUTPUT_SPEAKER,
								   AUDIOHAL_SPK_VOL_11,
								   25,
								   15);
    if(err != RTOS_SUCEESS) {
        LOG_E("config volume failed");
        return;
    }

    //配置非volte通话模式下, 耳机的侧音增益为MUTE(关闭侧音),且实时生效
	err = ql_aud_set_icsidet_gain(QL_AUD_VOICECALL_NB,
							QL_OUTPUT_HEADPHONE,
							QL_ICMIC_SIDET_GAIN_MUTE);
    if(err != RTOS_SUCEESS) {
        LOG_E("config side tone gain failed");
        return;
    }
    ql_aud_set_volume(QL_AUDIO_PLAY_TYPE_LOCAL, AUDIOHAL_SPK_VOL_6);
    err = def_rtos_queue_create(&audio_que_t, sizeof(uint8_t), 1);
    if(err != RTOS_SUCEESS) {
        LOG_E("audio_que_t is create failed");
        return;
    }
    sys_info.audio_init = 1;
    LOG_I("app_audio_init is ok");
}

void app_audio_thread(void *param)
{
    def_rtosStaus res;
    struct audio_con_stu audio_play;
    uint8_t type;
    app_audio_init();
    while(1) {
        // for(;;) {
        //     audio_play_test();
        // }
  //      LOG_I("IS RUN");
  
        res = def_rtos_queue_wait(audio_que_t, &type, sizeof(uint8_t), RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) continue;
        audio_play = audio_con_table[type];
        LOG_I("%s start, cnt:%d", audio_play.file_name, audio_play.play_cnt);
        play_flag = 0;
        for(;;) {
            LOG_I("audio_play.play_cnt:%d", audio_play.play_cnt);
            res = ql_aud_play_file_start(audio_play.file_name, QL_AUDIO_PLAY_TYPE_LOCAL, NULL);
            if(res) {
                LOG_E("ql_aud_play_file_start fail,res:%d", res);
                break;
            }
            res = ql_aud_wait_play_finish(QL_WAIT_FOREVER);
            if(res) {
                LOG_E("ql_aud_wait_play_finish,res:%d", res);
                break;
            }
            if(audio_play.play_cnt == 0xff) {
                def_rtos_task_sleep_ms(audio_play.play_interval);
            } else {
                audio_play.play_cnt--;
                if(audio_play.play_cnt == 0) {
                    LOG_I("audio_play.play_cnt:%d", audio_play.play_cnt);
                    break;
                }
                def_rtos_task_sleep_ms(audio_play.play_interval);
            } 
            if(play_flag == 1) break;
            LOG_I("audio play:%s", audio_play.file_name);
        }
        LOG_I("play_flag:%d, audio_play.play_cnt:%d, res:%d, aud is quit", play_flag, audio_play.play_cnt, res);    
    }
    def_rtos_task_delete(NULL);
}