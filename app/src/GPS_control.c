#include "app_system.h"
#include "ringbuffer.h"

#define DBG_TAG         "GPS_control"

#ifdef GPS_CONTROL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"
#define GPS_POS_CNT  30

struct rt_ringbuffer *GPS_Ringbuf;
#define GPS_DATA_LEN       512
int64_t gps_resh_time_t;
def_rtos_sem_t gps_sem;
GPS_DATA GpsDataBuf; 
GPS_INFO  Gps;



double Convert_LngLat(double val)
{
    uint32_t u_val;
    u_val = (uint32_t)val;
    return (u_val / 100) + ((u_val % 100) + (val - u_val)) / 60;
}

void GPS_Init()
{
    def_rtosStaus err = RTOS_SUCEESS;
    err = def_rtos_semaphore_create(&gps_sem, 0);
    if(err != RTOS_SUCEESS) {
        LOG_E("gps_sem is error");
    }
    GPS_Ringbuf = rt_ringbuffer_create(GPS_DATA_LEN);
	gps_resh_time_t = 0;
}

void gps_data_trans(uint8_t *data, uint16_t len)
{
    rt_ringbuffer_put(GPS_Ringbuf, data, len);
    if(strstr((char *)data, "RMC")) {
        def_rtos_smaphore_release(gps_sem);
    }
}


uint16_t gps_data_block_recv(uint8_t *data, uint16_t len, uint32_t time)
{
    uint16_t buf_len, read_len;
    if(RTOS_SUCEESS == def_rtos_semaphore_wait(gps_sem, time)){
        buf_len = rt_ringbuffer_data_len(GPS_Ringbuf);
        read_len = MIN(buf_len, len);
        rt_ringbuffer_get(GPS_Ringbuf, data, read_len);
        return read_len;
    } else return 0;
}

uint8_t  GPS_LngLat_Proces(char *Data, GPS_DATA  *pGpsData)
{
    char *pPara[16], *pStart;
    uint8_t EndFlag;
    const uint8_t ParaNum = 4;                           //½âÎöµÄ²ÎÊý¸öÊý


    char *ps = strstr(Data, "N");
    if(ps == NULL)    return FAIL;

    ps = strstr(Data, "E");
    if(ps == NULL)    return FAIL;


    pStart = Data;                 //²éÕÒÐ£ÑéÊý¾Ý
    if(pStart == NULL)
        return FAIL;

    memset(pPara, 0, sizeof(pPara));
    if(String_Para_Intercept(pStart, pPara, ParaNum, &EndFlag, ',', '*') != ParaNum)
        return FAIL;

    if(pPara[0] != NULL)
        pGpsData->Latitude  = strtod(pPara[0], NULL);        //Latitude Î³¶ÈÐÅÏ¢   ¶È·Ö¸ñÊ½

    if(pPara[2] != NULL)
        pGpsData->Longitude = strtod(pPara[2], NULL);        //Longitude ¾­¶ÈÐÅÏ¢  ¶È·Ö¸ñÊ½

    #ifdef DUBUG_GPS_FLAG
    #if 1
    printf("\n\n***GNSS_PQTM_Proces*****\n");
    printf("Latitude:%lf\n", pGpsData->Latitude);
    printf("Longitude:%lf\n\n", pGpsData->Longitude);
    #endif
    #endif

    return OK;
}

uint8_t  GPS_Number_Cheek(char *DataStr)
{
	#if 0 //²»Ð£Ñé 2023-6-8 17:08:49
    uint8_t  Len;
    uint16_t i;

    Len = strlen(DataStr);
    for(i = 0; i < Len; i++)
    {
        if(((DataStr[i] < '0') || (DataStr[i] > '9')) && (DataStr[i] != '.'))
            return RT_FAIL;
    }
	#endif
	
    return OK;
}

uint8_t  GPS_RMC_Proces(char *Data, GPS_DATA  *pGpsData)
{
    char Temp;
    uint8_t  DataLen, i;
    char *ps, *pe, Buf[64], LatBuf[64];

    LOG_I("**********GPS_RMC_Proces***********");
    //	        0  1           2  3            4  5      6 7       8  9 10   11
    //063717.00, A, 2238.07773, N, 11407.55384, E, 0.048, , 281217,  ,  ,  A *
    ps = Data;                                       
 //   LOG_I("%s", ps);
    for(i = 0; i < 14; i++)
    {
        pe = strstr(ps, ",");
        if(pe == NULL)
        {
            pe = strstr(ps, "*");                       
            if(pe == NULL) {
                return FAIL;
            }
        }

        #if  0                                         
        if(pe == ps)                                   //ÎÞÊý¾Ý
        {
            ps = pe + 1;                                //ÖØÐÂ¸³ÖµÆðÊ¼µØÖ·
            continue;
        }
        #endif


        DataLen = pe - ps;
        memset(Buf, 0, sizeof(Buf));
        if(DataLen < sizeof(Buf))	
		{
			memcpy(Buf, ps, DataLen); 
		}
        else {
            return FAIL;    
        }                       

     //   LOG_I("i:%d  DataLen:%d  Buf:%s\n", i, DataLen, Buf);
		
        switch(i)
        {
            case 1:                                        
                if((DataLen >= sizeof(pGpsData->Time1)) || (GPS_Number_Cheek(Buf) != OK)) 
                    return FAIL;
                strcpy(pGpsData->Time1, Buf);
                break;

            case 2:
                if(Buf[0] == 'A')  pGpsData->GPSValidFlag = 1;
                break;  
            case 3:
                memset(LatBuf, 0, sizeof(LatBuf));
            case 4:
            case 5:
            case 6:
                strcat(LatBuf, Buf);                            
                if(i == 6)
                {
                    if(strlen(LatBuf) >= sizeof(pGpsData->LatLongData))
					{
                        return FAIL;
					}
                    strcpy(pGpsData->LatLongData, LatBuf);    
                }
                else
                {
                    if((i == 3) || (i == 5))                         
                    {
                        if(GPS_Number_Cheek(Buf) != OK)
						{
                            return FAIL;
						}
                    }
                    strcat(LatBuf, ",");
                }
                break;
			case 7:
				strcpy(pGpsData->Ground_speed, Buf);
				break;
			case 8:
				strcpy(pGpsData->Yaw, Buf);
				break;
            case 9:                                              
                strcpy(pGpsData->Time2, Buf);
                break;
            case 12:
                Temp = Buf[0];
                if((Temp == 'A') || (Temp == 'D') || (Temp == 'E') || (Temp == 'N'))                       
                {
                    pGpsData->Mode[0] = Temp;
                    return OK;
                }
                return FAIL;

            default:
                break;
        }
        ps = pe + 1;                                       
    }
    return OK;
}

uint8_t  GPS_GGA_Proces(char *Data, GPS_DATA  *pGpsData)
{
    uint8_t  DataLen, i;
    char *ps, *pe, Buf[64], LatBuf[64];

    LOG_I("\n**********GPS_GGA_Proces***********\n");

    //	        0           1  2            3  4  5   6     7      8  9     10 11 12  13
    //063717.00, 2238.07773, N, 11407.55384, E, 1, 09, 1.02, 101.3,  M, -2.2, M,  ,  *
    //$GNGGA,160957.00,,,,,0,00,99.99,,,,,,*74
    ps = Data;                                       
    for(i = 0; i < 12; i++)
    {
        pe = strstr(ps, ",");
        if(pe == NULL)
        {
            pe = strstr(ps, "*");                      
            if(pe == NULL)  return FAIL;
        }

        #if 0                                         
        if(pe == ps)                                  
        {
            ps = pe + 1;                             
            continue;
        }
        #endif


        DataLen = pe - ps;
        memset(Buf, 0, sizeof(Buf));
        if(DataLen < sizeof(Buf))	memcpy(Buf, ps, DataLen); 
        else return -1;                          

    //    LOG_I("i:%d  DataLen:%d  Buf:%s\n", i, DataLen, Buf);

        switch(i)
        {
            case 1:                                        
                if((DataLen >= sizeof(GpsDataBuf.Time1)) || (GPS_Number_Cheek(Buf) != OK)) 
                    return -1;
                strcpy(pGpsData->Time1, Buf);
                break;

            case 2:
                memset(LatBuf, 0, sizeof(LatBuf));
            case 3:
            case 4:
            case 5:
                strcat(LatBuf, Buf);                       // ddmm.mmmm,N/S,dddmm.mmmm,E/W  2238.07773,N,11407.55384,E;
                if(i == 5)
                {
                    if(strlen(LatBuf) >= sizeof(GpsDataBuf.LatLongData))
                        return FAIL;
                    strcpy(pGpsData->LatLongData, LatBuf);       
                }
                else
                {
                    if((i == 2) || (i == 4))                          
                    {
                        if(GPS_Number_Cheek(Buf) != OK)
                            return FAIL;
                    }
                    strcat(LatBuf, ",");
                }
                break;

            case 6:
                if(Buf[0] == '0')   pGpsData->Mode[0] = 'N';      
                else if(Buf[0] == '2')   pGpsData->Mode[0] = 'D'; 
                else if(Buf[0] == '6')   pGpsData->Mode[0] = 'E'; 
                else pGpsData->Mode[0] = 'A';                     

                if(pGpsData->Mode[0] != 'N')
                    pGpsData->GPSValidFlag = 1;
                break;

            case 7:                                            
                if((DataLen >= sizeof(GpsDataBuf.SateNumStr)) || (GPS_Number_Cheek(Buf) != OK)) 
                    return FAIL;
                strcpy(pGpsData->SateNumStr, Buf);
                break;

            case 8:                                              
                if((DataLen >= sizeof(GpsDataBuf.HDOP)) || (GPS_Number_Cheek(Buf) != OK)) 
                    return FAIL;
                strcpy(pGpsData->HDOP, Buf);
                break;

            case 9:                                              
                if((DataLen > 8) || (GPS_Number_Cheek(Buf) != OK)) 
                    return FAIL;
                strcpy(pGpsData->SeaLevelH, Buf);
                strcat(pGpsData->SeaLevelH, ",");
                break;

            case 10:
                if(strlen(Buf) <= 2)	                           
                {
                    strcat(pGpsData->SeaLevelH, Buf);
                    return OK;
                }
                else  return FAIL;

            default:
                break;
        }
        ps = pe + 1;                                      
    }
    return OK;
}

uint16_t  GPS_FloatStr_To_Num(char *Data)
{
    uint16_t  Num = 0;
    char Buf1[8], Buf2[8];
    //               ×Ö·û´®³¤¶È´íÎó                    ×Ö·û´®Îª¿Õ        ·µ»Ø×î´óÖµ
    if((strlen(Data) == 0) || (strlen(Data) >= 8) || (Data == NULL))   return 0xFFFF;

    memset(Buf1, 0, sizeof(Buf1));
    memset(Buf2, 0, sizeof(Buf2));
    while(*Data)
    {
        if(*Data == '.')
        {
            strcpy(Buf2, Data + 1);
            Num  = atoi(Buf1) * 100;
            if((Buf2[0] >= '0') && (Buf2[0] <= '9'))
                Num += (Buf2[0] - '0') * 10;
            if((Buf2[1] >= '0') && (Buf2[1] <= '9'))
                Num += Buf2[1] - '0';
            return Num;
        }
        else
        {
            Buf1[Num++] = *Data;
        }
        Data++;
    }
    return 0xFFFF;
}

uint8_t GPS_Data_Proces(char *data, uint16_t len)
{
    uint16_t  Acc1 = 0, Acc2 = 0;
    GPS_DATA  GData, GDataTemp;
    char *start;
    start = strstr(data, "RMC");
	if(start) {  
		memset(&GDataTemp, 0, sizeof(GDataTemp));
        if(GPS_RMC_Proces(start, &GDataTemp) == OK) {
			GData.GpsFlag = 1;
			GData.GPSValidFlag = GDataTemp.GPSValidFlag;
            memcpy(GData.Time1, GDataTemp.Time1, sizeof(GData.Time1));
            memcpy(GData.LatLongData, GDataTemp.LatLongData, sizeof(GData.LatLongData));
            memcpy(GData.Time2, GDataTemp.Time2, sizeof(GData.Time2));
            memcpy(GData.Mode, GDataTemp.Mode, sizeof(GData.Mode));
			memcpy(GData.Ground_speed, GDataTemp.Ground_speed, sizeof(GData.Ground_speed));
			memcpy(GData.Yaw, GDataTemp.Yaw, sizeof(GData.Yaw));

			#ifdef USE_LOCAL_GEOFENCE   
			if(GPS_LngLat_Proces(GDataTemp.LatLongData, &GData) == OK) {
				GpsDataBuf.Latitude = Convert_LngLat(GData.Latitude);             		
                GpsDataBuf.Longitude = Convert_LngLat(GData.Longitude);
				if((GpsDataBuf.Longitude != 0) && (GpsDataBuf.Latitude != 0))
				{
					GpsDataBuf.RefreshFlag = 1;
				}	
			}
			#endif
            LOG_I("\n**********RT_OK***********\n");
            LOG_I("GPSValidFlag:%d", GDataTemp.GPSValidFlag);
            LOG_I("Time1:%s", GDataTemp.Time1);
            LOG_I("LatLongData:%s", GDataTemp.LatLongData);
            LOG_I("Time2:%s", GDataTemp.Time2);
            LOG_I("Mode:%s", GDataTemp.Mode);
            LOG_I("Ground_speed:%s", GDataTemp.Ground_speed);
            LOG_I("Yaw:%s", GData.Yaw);
		}
    } 
    start = strstr(data, "GGA");
    if(start) {
		memset(&GDataTemp, 0, sizeof(GDataTemp));
        if(GPS_GGA_Proces(start, &GDataTemp) == OK) {
            GData.GpsFlag = 1;   //获取有效数据
            GData.GPSValidFlag = GDataTemp.GPSValidFlag;
            memcpy(GData.Time1, GDataTemp.Time1, sizeof(GData.Time1));
            memcpy(GData.LatLongData, GDataTemp.LatLongData, sizeof(GData.LatLongData));
            memcpy(GData.Mode, GDataTemp.Mode, sizeof(GData.Mode));
            memcpy(GData.SateNumStr, GDataTemp.SateNumStr, sizeof(GData.SateNumStr));
            memcpy(GData.HDOP, GDataTemp.HDOP, sizeof(GData.HDOP));
            memcpy(GData.SeaLevelH, GDataTemp.SeaLevelH, sizeof(GData.SeaLevelH));

            LOG_I("**********RT_OK***********");
            LOG_I("GPSValidFlag:%d", GDataTemp.GPSValidFlag);
            LOG_I("Time1:%s", GDataTemp.Time1);
            LOG_I("LatLongData:%s", GDataTemp.LatLongData);
            LOG_I("Mode:%s", GDataTemp.Mode);
            LOG_I("SateNumStr:%s", GDataTemp.SateNumStr);
            LOG_I("HDOP:%s", GDataTemp.HDOP);
            LOG_I("SeaLevelH:%s", GDataTemp.SeaLevelH);
        }
	}
   
    if(GData.GpsFlag == 0)  return FAIL;

    if(GData.GPSValidFlag == 1) {
        if(strlen(GData.LatLongData) <= 5)
            GData.GPSValidFlag = 0;
    } else {
        if(strlen(GData.LatLongData) <= 5) {
            if(GpsDataBuf.GPSValidFlag == 0) 
                memcpy(&GpsDataBuf, &GData, sizeof(GPS_DATA));
        }
        else GData.GPSValidFlag = 1;
    }

    if(GData.GPSValidFlag == 0)  {
        if(Gps.GpsMode == GPS_MODE_CONT)                           
            memcpy(&GpsDataBuf, &GData, sizeof(GPS_DATA));
        return  FAIL;
    }
    if(strlen(GData.SateNumStr) != 0)  Gps.SateNum = atoi(GData.SateNumStr);
    if(GpsDataBuf.GPSValidFlag == 1) {
        if(strlen(GData.Time1) == 0)  strcpy(GData.Time1, GpsDataBuf.Time1);
        if(strlen(GData.Time2) == 0)  strcpy(GData.Time2, GpsDataBuf.Time2);
        if(strlen(GData.HDOP) == 0)  strcpy(GData.HDOP, GpsDataBuf.HDOP);
        if(strlen(GData.SeaLevelH) == 0)  strcpy(GData.SeaLevelH, GpsDataBuf.SeaLevelH);
        if(strlen(GData.Mode) == 0)  strcpy(GData.Mode, GpsDataBuf.Mode);	
        if(strlen(GData.SateNumStr) == 0)  strcpy(GData.SateNumStr, GpsDataBuf.SateNumStr);
        if(Gps.GpsMode == GPS_MODE_TM) {
             if((strlen(GData.HDOP) != 0) && (strlen(GpsDataBuf.HDOP) != 0)) {
                Acc1 = GPS_FloatStr_To_Num(GData.HDOP);
                Acc2 = GPS_FloatStr_To_Num(GpsDataBuf.HDOP);
                if(Acc1 > Acc2)
                    strcpy(GData.LatLongData, GpsDataBuf.LatLongData);
             }
        }
    } else {
        if(strlen(GData.Time1) == 0)  strcpy(GData.Time1, GpsDataBuf.Time1);
        if(strlen(GData.Time2) == 0)  strcpy(GData.Time2, GpsDataBuf.Time2);
        if(strlen(GData.SeaLevelH) == 0)  strcpy(GData.SeaLevelH, GpsDataBuf.SeaLevelH);
    }
    memcpy(&GpsDataBuf, &GData, sizeof(GPS_DATA)); 
    return OK;
}

static void GPS_Composite_PosData()
{
    uint8_t Len;
    #ifdef OM_NET_PROTOCOL
    if(GpsDataBuf.GpsFlag == 0){
        strcpy(Gps.PosData, ",V,,,,,,,,,,N");
        return;
    }
    memset(Gps.PosData, 0, sizeof(Gps.PosData));
    Len = sprintf(Gps.PosData, "%s,", GpsDataBuf.Time1);
    if(GpsDataBuf.GPSValidFlag == 1) {
        Len += sprintf(Gps.PosData + Len, "%c,", 'A');
    } else{
        Len += sprintf(Gps.PosData + Len, "%c,", 'V');
    }

	if(strlen(GpsDataBuf.LatLongData) == 0)           
		strcpy(GpsDataBuf.LatLongData, ",,,");

	if(strlen(GpsDataBuf.SeaLevelH) == 0)           
            strcpy(GpsDataBuf.SeaLevelH, ",");	
	
    Len += sprintf(Gps.PosData + Len, "%s,%s,%s,", GpsDataBuf.LatLongData, GpsDataBuf.SateNumStr, GpsDataBuf.HDOP);
    Len += sprintf(Gps.PosData + Len, "%s,%s,%s", GpsDataBuf.Time2, GpsDataBuf.SeaLevelH, GpsDataBuf.Mode);
    #endif
}

void GPS_stop()
{
    MCU_CMD_MARK(CMD_GPS_POWEROFF_INDEX);
    Gps.GpsPower = GPS_POWER_OFF;
}
void GPS_Up_Pos()
{
    
}

void GPS_Start(uint8_t Mode)
{
    if(Gps.GpsPower != GPS_POWER_ON) {
        MCU_CMD_MARK(CMD_GPS_POWERON_INDEX);
        Gps.GpsMode = Mode;
        Gps.GetGPSNum = 0; 
        memset(&GpsDataBuf, 0, sizeof(GpsDataBuf)); 
        strcpy(Gps.PosData, ",V,,,,,,,,,,N");  
        if(Mode == GPS_MODE_TM) {
            Gps.Gps_Tm_timeout = def_rtos_get_system_tick();
        }
    } else if(Mode == GPS_MODE_TM) {
        if(Gps.GpsMode == GPS_MODE_CONT)
        {
            GPS_Up_Pos();                                      //ÉÏ´«¶¨Î»
        }
    } else if(Mode == GPS_MODE_CONT) {
        if(Gps.GpsMode != GPS_MODE_CONT) {
            Gps.GpsMode = GPS_MODE_CONT;
        }
    }
}

void gps_control_thread(void *param)
{
    uint8 gps_buf[256];
    uint16_t len;
    while(1){
        len = gps_data_block_recv(gps_buf, 256, RTOS_WAIT_FOREVER);
        if(len == 0) continue;
		if(GPS_Data_Proces((char *)gps_buf, len) == OK) {
            Gps.GetGPSNum++;
        } 
        if(Gps.GpsMode == GPS_MODE_TM) {
            if(Gps.GetGPSNum >= GPS_POS_CNT || systm_tick_diff(Gps.Gps_Tm_timeout) > 180*1000) {
                GPS_Composite_PosData();
                GPS_stop();
                if(Gps.GetGPSNum >= GPS_POS_CNT) {
                    LOG_I("**********GPS OK***********");
                } else {
                    LOG_E("**********GPS Time Out***********");
                }
                continue;
            }
        } else {
            GPS_Composite_PosData(); 
        }
    }
    def_rtos_task_delete(NULL);
}

