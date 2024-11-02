#ifndef   __APP_AUDIO_H
#define   __APP_AUDIO_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#define   LOCK_VOICE_FILE                   "lock.wav"
#define   UNLOCK_VOICE_FILE                 "unlock.wav"
#define   ALARM_VOICE_FILE                  "alarm.wav"
#define   LOOK_CAR_VOICE_FILE               "look_car.wav"
#define   ENTER_PENALTY_AREA_VOICE_FILE     "enter_penalty_area.wav"
#define   ENTER_FEASIBLE_AREA_VOICE_FILE    "enter_feasible_area.wav"  




typedef enum {
    LOCK_VOICE = 0,
    UNLOCK_VOICE,
    ALARM_VOICE,
    LOOK_CAR_VOICE,
    ENTER_PENALTY_AREA_VOICE,
    ENTER_FEASIBLE_AREA_VOICE
} VOICE_TYPE;














#ifdef __cplusplus
}
#endif

#endif