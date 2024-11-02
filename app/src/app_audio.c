#include    "app_system.h"
#include    "ql_audio.h"

#define DBG_TAG         "app_audio"

#ifdef APP_AUDIO_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

def_rtos_sem_t audio_sem_t;

struct audio_con_stu
{
    uint8_t play_cnt;
    uint32_t play_interval;
    char file_name[56];
};



struct audio_con_stu audio_con_table[] = {
    {1,     0,      LOCK_VOICE_FILE},
    {1,     0,      UNLOCK_VOICE_FILE},
    {-1,    2000,   ALARM_VOICE_FILE},
    {-1,    2000,   LOOK_CAR_VOICE_FILE},
    {3,     1000,   ENTER_PENALTY_AREA_VOICE_FILE},
    {3,     1000,   ENTER_FEASIBLE_AREA_VOICE_FILE},
};

struct audio_con_stu audio_play;
uint8_t play_flag;
void voice_play_mark(VOICE_TYPE type)
{
    AudioStatus_e audio_state;
    audio_state = ql_aud_get_play_state();
    if(audio_state != QL_AUDIO_STATUS_IDLE){
        ql_aud_player_stop();
    }
    audio_play = audio_con_table[type];
    def_rtos_smaphore_release(audio_sem_t);
    play_flag = 1;
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

    def_rtos_semaphore_create(&audio_sem_t, 0);
}

void app_audio_thread(void *param)
{
    def_rtosStaus res;
    while(1) {
        res = def_rtos_semaphore_wait(audio_sem_t, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) continue;
        play_flag = 0;
        for(;;) {
            res = ql_aud_play_file_start(audio_play.file_name, QL_AUDIO_PLAY_TYPE_LOCAL, NULL);
            if(res) break;
            ql_aud_wait_play_finish(QL_WAIT_FOREVER);
            if(audio_play.play_cnt == -1) {
                def_rtos_task_sleep_ms(audio_play.play_interval);
            } else {
                audio_play.play_cnt--;
                if(audio_play.play_cnt == 0) break;
                def_rtos_task_sleep_ms(audio_play.play_interval);
            } 
            if(play_flag == 1) break;
        }
    }
    def_rtos_task_delete(NULL);
}