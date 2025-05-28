#include "app_system.h"
#include "ringbuffer.h"
#include <math.h>

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


#define EARTH_RADIUS 6378.137
Point test_p[5] = {{23, 100}, {30, 120}, {25, 140}, {18,130}, {15, 110}};

static double radian(double d)
{
    return d*M_PI/180;
}

static double get_distance(Point p1, Point p2)
{
    double radLat1 = radian(p1.lat);
    double radLat2 = radian(p2.lat);

    double a = radLat1 - radLat2;
    double b = radian(p1.lon) - radian(p2.lon);
    double dst = 2*asin((sqrt(pow(sin(a/2), 2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2), 2))));
    dst = dst *EARTH_RADIUS;
    dst = round(dst *10000)/10000;
    return dst;             
}
/*
radius:千米
*/
static int isInner_circle(Point center, double radius, Point p)
{
    double dts;
    dts = get_distance(center, p);
    return dts < radius;
}
static int isInner_polygon(Point p, Point *Poly, int num)  //多边形禁区
{
    int flag = 0, i;
    for(i = 0; i < num; i++){
        Point p1 = Poly[i];
        Point p2 = Poly[(i + 1)%num];
        if((p.lat >= p1.lat && p.lat< p2.lat) || (p.lat >= p2.lat && p.lat < p1.lat)){
            double lon = (p.lat - p1.lat)*(p2.lon - p1.lon)/(p2.lat - p1.lat) + p1.lon;
            if(lon > p.lon) {
                flag++;
            }
        }
    }
    return flag%2;
}


void electron_fence_test()
{
    Point p_t = {25, 160};
    Point p1 = {0,110.12}, p2 = {0, 111.12};
    double dst;
    int res;
    res = isInner_polygon(p_t, test_p, 5);
    LOG_I("RES:%d", res);
    res = isInner_circle(p1, 20.0, p2);
    LOG_I("RES:%d", res);
    dst = get_distance(p1, p2);
    LOG_I("DISTANCE:%0.2f", dst);

}

double Convert_LngLat(double val)
{
    uint32_t u_val;
    u_val = (uint32_t)val;
    return (u_val / 100) + ((u_val % 100) + (val - u_val)) / 60;
}

int Convert_LonLat(GPS_DATA *gps_data)
{
    char str[4][20] = {0};
    char *p1, *p2;
    char *dec_p;
    double lat, lon;
    double degree;

    if(strlen(gps_data->LatLongData) < 5) return FAIL;

    p1 = gps_data->LatLongData;
    for(int i = 0; i < 3; i++) {
       p2 = strchr(p1, ',');
       memcpy(str[i], p1, p2 - p1);
       p1 = p2+1;
    }
    str[3][0] = *p1;
    dec_p = strchr(str[0], '.');
    if(dec_p == NULL)return FAIL;
    
    lon = strtod(str[0], NULL);
    degree = (lon - (int)(lon/100)*100) /60.0;
    lon = (int)(lon/100)+ degree;
    if(str[1][0] == 'S'){
        gps_data->Longitude = -lon;
    } else {
        gps_data->Longitude = lon;
    }

    lat = strtod(str[2], NULL);
    degree = (lat - (int)(lat/100)*100) /60.0;
    lat = (int)(lat/100)+ degree;
    if(str[3][0] == 'W') {
        gps_data->Latitude = -lat;
    } else{
        gps_data->Latitude = lat;
    }
    LOG_I("lon:%f, lat:%f", gps_data->Longitude, gps_data->Latitude);
    return OK;
}

void GPS_Init()
{
    def_rtosStaus err = RTOS_SUCEESS;
    err = def_rtos_semaphore_create(&gps_sem, 0);
    if(err != RTOS_SUCEESS) {
        LOG_E("gps_sem is error");
    }
    memset(&Gps, 0, sizeof(Gps));
    GPS_Ringbuf = rt_ringbuffer_create(GPS_DATA_LEN);
	gps_resh_time_t = 0;
    Gps.GpsPower = GPS_POWER_OFF;
    GPS_Start(GPS_MODE_TM);
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
                if((Temp == 'A') || (Temp == 'D') || (Temp == 'E') || (Temp == 'N'))    /*N:表示无定位*/                   
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
                pGpsData->high = strtod(Buf, NULL);
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
    // uint16_t  Acc1 = 0, Acc2 = 0;
    GPS_DATA  GData, GDataTemp;
    char *start;
    memset(&GData, 0, sizeof(GData));
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
            if(Convert_LonLat(&GData) == OK) {
                GpsDataBuf.RefreshFlag = 1;
            }
            // LOG_I("\n**********RT_OK***********\n");
            // LOG_I("GPSValidFlag:%d", GDataTemp.GPSValidFlag);
            // LOG_I("Time1:%s", GDataTemp.Time1);
            // LOG_I("LatLongData:%s", GDataTemp.LatLongData);
            // LOG_I("Time2:%s", GDataTemp.Time2);
            // LOG_I("Mode:%s", GDataTemp.Mode);
            // LOG_I("Ground_speed:%s", GDataTemp.Ground_speed);
            // LOG_I("Yaw:%s", GDataTemp.Yaw);
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

            // LOG_I("**********RT_OK***********");
            // LOG_I("GPSValidFlag:%d", GDataTemp.GPSValidFlag);
            // LOG_I("Time1:%s", GDataTemp.Time1);
            // LOG_I("LatLongData:%s", GDataTemp.LatLongData);
            // LOG_I("Mode:%s", GDataTemp.Mode);
            // LOG_I("SateNumStr:%s", GDataTemp.SateNumStr);
            // LOG_I("HDOP:%s", GDataTemp.HDOP);
            // LOG_I("SeaLevelH:%s", GDataTemp.SeaLevelH);
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

    if(GData.GPSValidFlag == 0){
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
        if(strlen(GData.Ground_speed) == 0) strcpy(GData.Ground_speed, GpsDataBuf.Ground_speed);
        if(strlen(GData.Yaw) == 0) strcpy(GData.Yaw, GpsDataBuf.Yaw);
        if(Gps.GpsMode == GPS_MODE_TM) {
            //  if((strlen(GData.HDOP) != 0) && (strlen(GpsDataBuf.HDOP) != 0)) {
            //     Acc1 = GPS_FloatStr_To_Num(GData.HDOP);
            //     Acc2 = GPS_FloatStr_To_Num(GpsDataBuf.HDOP);
            //     if(Acc1 > Acc2)
            //         strcpy(GData.LatLongData, GpsDataBuf.LatLongData);
            //  }
        }
    } else {
        if(strlen(GData.Time1) == 0)  strcpy(GData.Time1, GpsDataBuf.Time1);
        if(strlen(GData.Time2) == 0)  strcpy(GData.Time2, GpsDataBuf.Time2);
        if(strlen(GData.SeaLevelH) == 0)  strcpy(GData.SeaLevelH, GpsDataBuf.SeaLevelH);
    }
    if(GData.GPSValidFlag == 1 && GpsDataBuf.GPSValidFlag == 1){
        memcpy(&GpsDataBuf, &GData, sizeof(GPS_DATA));  
        LOG_I("GPS UPDATE"); 
        // if(distance > 0.2) {
        //     memcpy(&GpsDataBuf, &GData, sizeof(GPS_DATA)); 
        // }
        // else if(car_info.speed != 0 && car_info.lock_sta == CAR_UNLOCK_STA) //车如果不是静止状态，运动时更新位置
        // {
        //     memcpy(&GpsDataBuf, &GData, sizeof(GPS_DATA)); 
        // } 
    } else if(GData.GPSValidFlag == 1 && GpsDataBuf.GPSValidFlag == 0) {
        memcpy(&GpsDataBuf, &GData, sizeof(GPS_DATA)); 
    }
    return OK;
}
static void GPS_Composite_PosData()
{

    #ifndef OM_NET_PROTOCOL
    uint8_t Len;
    if(GpsDataBuf.GpsFlag == 0){
        strcpy(Gps.PosData, ",V,,,,,,,,,,N");
        return;
    }
    memset(Gps.PosData, 0, sizeof(Gps.PosData));
    Len = sprintf(Gps.PosData, "%s,", GpsDataBuf.Time1);
    if(GpsDataBuf.GPSValidFlag == 1) {
        Len += sprintf(Gps.PosData + Len, "%c,", 'A');
    } else {
        Len += sprintf(Gps.PosData + Len, "%c,", 'V');
    }

	if(strlen(GpsDataBuf.LatLongData) == 0)           
		strcpy(GpsDataBuf.LatLongData, ",,,");

	if(strlen(GpsDataBuf.SeaLevelH) == 0)           
            strcpy(GpsDataBuf.SeaLevelH, ",");	
	
    Len += sprintf(Gps.PosData + Len, "%s,%s,%s,", GpsDataBuf.LatLongData, GpsDataBuf.SateNumStr, GpsDataBuf.HDOP);
    Len += sprintf(Gps.PosData + Len, "%s,%s,%s", GpsDataBuf.Time2, GpsDataBuf.SeaLevelH, GpsDataBuf.Mode);
    #else
    if(GpsDataBuf.GPSValidFlag == 0 || GpsDataBuf.GpsFlag == 0){
        Gps.vaild = 0;
        return;
    }

    Gps.vaild = 1;
    Gps.ground_speed = (uint16_t)(strtod(GpsDataBuf.Ground_speed, NULL)*10.0);
    LOG_I("ground_speed:%d", Gps.ground_speed);
    if(strtod(GpsDataBuf.HDOP, NULL)*10.0 > 255.0){
        Gps.hdop = 25;
    } else {
        Gps.hdop = strtod(GpsDataBuf.HDOP, NULL)*10.0;
    }
    Gps.direction = (uint16_t)(strtod(GpsDataBuf.Yaw, NULL)*10.0);
    LOG_I("direction:%d", Gps.direction);
    Gps.high = (uint16_t)GpsDataBuf.high;
    Gps.Lat = GpsDataBuf.Latitude*1000000;
    Gps.Long = GpsDataBuf.Longitude*1000000;
    LOG_I("statenum:%d", Gps.SateNum);
    #endif
}


void GPS_fence_detection()
{
    Point cur_p;
    if(car_info.lock_sta == CAR_LOCK_STA || sys_param_set.total_fence_sw == 0 ||(sheepfang_data.shape_type == FENCE_NONE && forbidden_zone_data.shape_type == FENCE_NONE)) {
        if(sys_info.voice_type_cur == ELECTRONIC_FENCE_VOICE) {
            voice_play_off();
        }
        return;
    }
    if(GpsDataBuf.GPSValidFlag == 0) return;    /*定位无效时不进行围栏检测*/
    cur_p.lat = GpsDataBuf.Latitude;
    cur_p.lon = GpsDataBuf.Longitude;
      /*羊圈检测*/   
    if(sheepfang_data.shape_type == CIRCLE) {
        if(isInner_circle(sheepfang_data.circle.center, sheepfang_data.circle.radius, cur_p) == 0) {  //在羊圈外 
            if(sys_set_var.sheepfang_flag == 0) {
                sys_set_var.sheepfang_flag = 1;  
                sys_info.sheepfang_sta = SHEEPFANG_LEAVE;
                LOG_I("SHEEPFANG_LEAVE");
                net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            } else {
                sys_info.sheepfang_sta = SHEEPFANG_OUT;
            }
        } else {
            if(sys_set_var.sheepfang_flag == 1) {
                sys_set_var.sheepfang_flag = 0;
                sys_info.sheepfang_sta = SHEEPFANG_ENTER;
                LOG_I("SHEEPFANG_ENTER");
                net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            } else {
                sys_info.sheepfang_sta = SHEEPFANG_IN;
            }
        }
      } else if(sheepfang_data.shape_type == POLYGON) {
        if(isInner_polygon(cur_p, sheepfang_data.polygon.p, sheepfang_data.polygon.point_num) == 0){
            if(sys_set_var.sheepfang_flag == 0) {
                sys_set_var.sheepfang_flag = 1;  
                sys_info.sheepfang_sta = SHEEPFANG_LEAVE;
                LOG_I("SHEEPFANG_LEAVE");
                net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            } else {
                sys_info.sheepfang_sta = SHEEPFANG_OUT;
            }
          } else {
              if(sys_set_var.sheepfang_flag == 1) {
                  sys_set_var.sheepfang_flag = 0;
                  sys_info.sheepfang_sta = SHEEPFANG_ENTER;
                  LOG_I("SHEEPFANG_ENTER");
                  net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
              } else {
                  sys_info.sheepfang_sta = SHEEPFANG_IN;
              }
          }
      } else {
          sys_info.sheepfang_sta = SHEEPFANG_INVALID;
      }
  
      /*禁区检测*/
      if(forbidden_zone_data.shape_type == CIRCLE) {
          if(isInner_circle(forbidden_zone_data.circle.center, forbidden_zone_data.circle.radius, cur_p) == 1){  //在禁区里
              if(sys_set_var.forbidden_flag == 0) {
                  sys_set_var.forbidden_flag = 1;  
                  sys_info.fence_sta= FORBIDDEN_ENTER;
                  LOG_I("FORBIDDEN_ENTER");
                  net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
              } else {
                  sys_info.fence_sta = FORBIDDEN_IN;
              }
          } else {
              if(sys_set_var.forbidden_flag == 1) {
                  sys_set_var.forbidden_flag = 0;
                  sys_info.fence_sta = FORBIDDEN_LEAVE;
                  LOG_I("FORBIDDEN_LEAVE");
                  net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
              } else {
                  sys_info.fence_sta = FORBIDDEN_OUT;
              }
          }
      } else if(forbidden_zone_data.shape_type == POLYGON) {
          if(isInner_polygon(cur_p, forbidden_zone_data.polygon.p, forbidden_zone_data.polygon.point_num) == 1){
              if(sys_set_var.forbidden_flag == 0) {
                  sys_set_var.forbidden_flag = 1;  
                  sys_info.fence_sta = FORBIDDEN_ENTER;
                  LOG_I("FORBIDDEN_ENTER");
                  net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
              } else {
                  sys_info.fence_sta = FORBIDDEN_IN;
              }
          } else {
              if(sys_set_var.forbidden_flag == 1) {
                  sys_set_var.forbidden_flag = 0;
                  sys_info.fence_sta = FORBIDDEN_LEAVE;
                  LOG_I("FORBIDDEN_LEAVE");
                  net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
              } else {
                  sys_info.fence_sta = FORBIDDEN_OUT;
              }
          }
      } else {
          sys_info.fence_sta = FORBIDDEN_INVALID;
      } 

      if(sys_info.sheepfang_sta == SHEEPFANG_LEAVE || sys_info.sheepfang_sta == SHEEPFANG_OUT || sys_info.fence_sta == FORBIDDEN_ENTER || sys_info.fence_sta == FORBIDDEN_IN) {
            if(sys_info.voice_type_cur != ELECTRONIC_FENCE_VOICE) {
                voice_play_mark(ELECTRONIC_FENCE_VOICE);
            }
      } else {
        if(sys_info.voice_type_cur == ELECTRONIC_FENCE_VOICE) {
            voice_play_off();
        }
      }
}

int GPS_reinit()
{
    static uint8_t step = 0;
    static int64_t gps_time_t = 0;
    if(Gps.init == 0) step = 0;
    switch (step) {
        case 0:
            Gps.init = 2;
            GPS_stop();
            step = 1;
            gps_time_t = def_rtos_get_system_tick();
        break;
        case 1:
            if(Gps.GpsPower == GPS_POWER_OFF) {
                step = 2;
                gps_time_t = def_rtos_get_system_tick();
            }
            else if(def_rtos_get_system_tick() - gps_time_t > 12000){
                step = 4;
            }
        case 2:
            if(def_rtos_get_system_tick() -  gps_time_t > 5000) {
                step = 3;
                GPS_Start(GPS_MODE_CONT);
                gps_time_t = def_rtos_get_system_tick();
            }
        break;
        case 3:
            if(Gps.GpsPower == GPS_POWER_ON) {
                LOG_I("GPS init ok");
                Gps.init = 1;
                return 0;
            } else if(def_rtos_get_system_tick() - gps_time_t > 12000) {
                step = 4;
            }
        break;
        case 4:
            LOG_E("GPS init fail");
            return 2;
        break;
    }
    return 1;
}

void GPS_stop()
{
    MCU_CMD_MARK(CMD_GPS_POWEROFF_INDEX);
    Gps.GpsMode = GPS_MODE_TM;
}


void GPS_Start(uint8_t Mode)
{
    if(Gps.GpsPower != GPS_POWER_ON) {
        MCU_CMD_MARK(CMD_GPS_POWERON_INDEX);
        Gps.GpsMode = Mode;
        Gps.GetGPSNum = 0; 
        memset(&GpsDataBuf, 0, sizeof(GpsDataBuf)); 
    //    strcpy(Gps.PosData, ",V,,,,,,,,,,N");  
        if(Mode == GPS_MODE_TM) {
            Gps.Gps_Tm_timeout = def_rtos_get_system_tick();
        }
    } else if(Mode == GPS_MODE_TM) {
        if(Gps.GpsMode == GPS_MODE_CONT)
        {
       //     NET_CMD_MARK(NET_CMD_Q_LOCATION_D0);
        }
    } else if(Mode == GPS_MODE_CONT) {
        if(Gps.GpsMode != GPS_MODE_CONT) {
            Gps.GpsMode = GPS_MODE_CONT;
        }
    }
}

typedef struct {
    double lat;     // 纬度
    double lon;     // 经度
    bool is_outlier;// 是否异常标记
} GpsPoint;

typedef struct {
    GpsPoint* data;
    int size;
    int capacity;
} GpsArray;

void detect_zscore(GpsArray* points, double threshold) {
    // 计算纬度/经度的均值和标准差
    double lat_sum = 0.0, lon_sum = 0.0;
    for (int i = 0; i < points->size; i++) {
        lat_sum += points->data[i].lat;
        lon_sum += points->data[i].lon;
    }
    double lat_mean = lat_sum / points->size;
    double lon_mean = lon_sum / points->size;

    double lat_std = 0.0, lon_std = 0.0;
    for (int i = 0; i < points->size; i++) {
        lat_std += pow(points->data[i].lat - lat_mean, 2);
        lon_std += pow(points->data[i].lon - lon_mean, 2);
    }
    lat_std = sqrt(lat_std / points->size);
    lon_std = sqrt(lon_std / points->size);

    // 标记异常点
    for (int i = 0; i < points->size; i++) {
        double z_lat = fabs((points->data[i].lat - lat_mean) / lat_std);
        double z_lon = fabs((points->data[i].lon - lon_mean) / lon_std);
        points->data[i].is_outlier = (z_lat > threshold || z_lon > threshold);
    }
}

#define GPS_CALC_CNT 25

int GPS_calcu_position(double lat, double lon)
{
    GpsArray points;
    static GpsPoint pos_table[GPS_CALC_CNT] = {0};
    static uint8_t pos_num = 0;
    double lat_sum = 0.0, lon_sum = 0.0;
    uint8_t out_num = 0;
    if(pos_num < GPS_CALC_CNT) {
        pos_table[pos_num].lat = lat;
        pos_table[pos_num].lon = lon;
        pos_table[pos_num].is_outlier = false;
        pos_num++;
        out_num = 0;
        lat_sum = 0.0;
        lon_sum = 0.0;
    } 

    if(pos_num == GPS_CALC_CNT) {
        points.data = pos_table;
        points.size = GPS_CALC_CNT;    
        points.capacity = GPS_CALC_CNT;
        detect_zscore(&points, 1.2);
        for (int i = 0; i < points.size; i++) {
            if(points.data[i].is_outlier == true) {
                LOG_I("gps position is false %d", i);
                out_num++;
            } else {
                lat_sum += pos_table[i].lat;
                lon_sum += pos_table[i].lon;
            }
        }
        if(out_num <= 1) {
            LOG_I("gps position is ok");
            GpsDataBuf.Latitude = lat_sum/(GPS_CALC_CNT - out_num);
            GpsDataBuf.Longitude = lon_sum/(GPS_CALC_CNT - out_num);
            pos_num = 0;
            return 0;
        } else {
            LOG_I("pos_table is move");
            memcpy(pos_table, pos_table+1, (GPS_CALC_CNT-1)*sizeof(GpsPoint));
            pos_num--;
        }
    }
    return 1;
}

void gps_control_thread(void *param)
{
    uint8 gps_buf[256];
    uint16_t len;
    GPS_DATA last_GpsDataBuf= {0};
    double gps_speed=0;
    double distance =0;
    Point P_last={0}, P_now={0};
    uint8_t s_cent = 0, d_cent = 0;
    while(1){
  //      LOG_I("IS RUN");
        len = gps_data_block_recv(gps_buf, 256, RTOS_WAIT_FOREVER);
        if(len == 0) continue;
        gps_resh_time_t = def_rtos_get_system_tick();
        if(iot_error_check(IOT_ERROR_TYPE, GPS_ERROR) == 1) {
            iot_error_clean(IOT_ERROR_TYPE, GPS_ERROR);
        }
		if(GPS_Data_Proces((char *)gps_buf, len) == OK) {
            Gps.GetGPSNum++;
        } 
        gps_speed = strtod(GpsDataBuf.Ground_speed, NULL);
        if(Gps.GpsMode == GPS_MODE_TM) {
            LOG_I("GetGPSNum:%d", Gps.GetGPSNum);
            if(Gps.GetGPSNum >= GPS_POS_CNT && Gps.GetGPSNum%5 == 0 && gps_speed == 0) {
                if(GPS_calcu_position(GpsDataBuf.Latitude, GpsDataBuf.Longitude) == 0) {
                    LOG_I("GPS position is ok");
                    GPS_Composite_PosData();
                    if(sys_info.paltform_connect) {         /*上传一次定位*/
                        NET_ENGWE_CMD_MARK(REGULARLY_REPORT_UP);
                    }
                    GPS_stop();
                } 
                else if (systm_tick_diff(Gps.Gps_Tm_timeout) > 600*1000) {
                    LOG_I("GPS position is fail");
                    GPS_Composite_PosData();
                    if(sys_info.paltform_connect) {         /*上传一次定位*/
                        NET_ENGWE_CMD_MARK(REGULARLY_REPORT_UP);
                    }
                    GPS_stop();
                }
            } 
        } else {
            if(GpsDataBuf.GPSValidFlag == 1 && last_GpsDataBuf.GPSValidFlag == 1) {
                P_now.lat = GpsDataBuf.Latitude;
                P_now.lon = GpsDataBuf.Longitude;
                P_last.lat = last_GpsDataBuf.Latitude;
                P_last.lon = last_GpsDataBuf.Longitude;
                distance = get_distance(P_now, P_last);
                if(distance > 0.2) {
                    if(++d_cent >= 10){
                        GPS_Composite_PosData();
                        d_cent = 0;
                    }
                } else if(gps_speed > 2.0) {
                    if (++s_cent > 5) {
                        GPS_Composite_PosData();
                        s_cent = 0;
                    }    
                } else {
                    d_cent = 0;
                    s_cent = 0;
                }
            } 
            if(car_info.speed != 0){
                GPS_Composite_PosData(); 
            }
            last_GpsDataBuf = GpsDataBuf;  
        }
    }
    def_rtos_task_delete(NULL);
}

