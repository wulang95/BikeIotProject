#ifndef   __APP_AUDIO_H
#define   __APP_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#define   LOCK_VOICE_FILE                   "close_lock.wav"
#define   UNLOCK_VOICE_FILE                 "open_lock.wav"
#define   ALARM_VOICE_FILE                  "alarm.wav"
#define   LOOK_CAR_VOICE_FILE               "look_car.mp3"
#define   ENTER_PENALTY_AREA_VOICE_FILE     "alarm.wav"
#define   DEFINE_VOICE_FILE                 "unfine_file.wav"



typedef enum {
    LOCK_VOICE = 0,
    UNLOCK_VOICE,
    ALARM_VOICE,
    LOOK_CAR_VOICE,
    ELECTRONIC_FENCE_VOICE,
    VOICE_TEST,
} VOICE_TYPE;





void voice_play_mark(VOICE_TYPE type);
void voice_play_off();
void app_audio_init();
void app_audio_thread(void *param);
void audio_dft_start();





#ifdef __cplusplus
}
#endif

#endif