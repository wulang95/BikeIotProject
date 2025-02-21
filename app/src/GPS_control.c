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
    char temp[6] = {0};
    char lat[20] ={0}, lon[20] = {0};
    unsigned long long degree = 0;
    int temp_i;
    char centPart[16] = {0};
    char *centDecimal = NULL;

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
    centDecimal = dec_p + 1;
    memcpy(temp, str[0], dec_p - str[0]);
    temp_i = atoi(temp);

    sprintf(centPart, "%d%s", temp_i%100, centDecimal);
    degree = atoll(centPart)*100/60;
    sprintf(lon, "%d.%llu", temp_i/100, degree);
    if(str[1][0] == 'S'){
        gps_data->Longitude = -strtod(lon,NULL);
    } else {
        gps_data->Longitude = strtod(lon, NULL);
    }

    memset(temp, 0, 6);
    dec_p = strchr(str[2], '.');
    if(dec_p == NULL) return FAIL;
    centDecimal = dec_p + 1;
    memcpy(temp, str[2], dec_p - str[0]);
    temp_i = atoi(temp);

    memset(centPart, 0, sizeof(centPart));
    sprintf(centPart, "%d%s", temp_i%100, centDecimal);
    degree = atoll(centPart)*100/60;
    sprintf(lat, "%d.%llu", temp_i/100, degree);
    if(str[3][0] == 'W') {
        gps_data->Latitude = -strtod(lat, NULL);
    } else{
        gps_data->Latitude = strtod(lat, NULL);
    }
    return OK;
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
 //   uint16_t  Acc1 = 0, Acc2 = 0;
    GPS_DATA  GData, GDataTemp;
    double distance;
    Point P1, P2;
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
            LOG_I("\n**********RT_OK***********\n");
            LOG_I("GPSValidFlag:%d", GDataTemp.GPSValidFlag);
            LOG_I("Time1:%s", GDataTemp.Time1);
            LOG_I("LatLongData:%s", GDataTemp.LatLongData);
            LOG_I("Time2:%s", GDataTemp.Time2);
            LOG_I("Mode:%s", GDataTemp.Mode);
            LOG_I("Ground_speed:%s", GDataTemp.Ground_speed);
            LOG_I("Yaw:%s", GDataTemp.Yaw);
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
        // if(Gps.GpsMode == GPS_MODE_TM) {
        //      if((strlen(GData.HDOP) != 0) && (strlen(GpsDataBuf.HDOP) != 0)) {
        //         Acc1 = GPS_FloatStr_To_Num(GData.HDOP);
        //         Acc2 = GPS_FloatStr_To_Num(GpsDataBuf.HDOP);
        //         if(Acc1 > Acc2)
        //             strcpy(GData.LatLongData, GpsDataBuf.LatLongData);
        //      }
        // }
    } else {
        if(strlen(GData.Time1) == 0)  strcpy(GData.Time1, GpsDataBuf.Time1);
        if(strlen(GData.Time2) == 0)  strcpy(GData.Time2, GpsDataBuf.Time2);
        if(strlen(GData.SeaLevelH) == 0)  strcpy(GData.SeaLevelH, GpsDataBuf.SeaLevelH);
    }
    if(GData.GPSValidFlag == 1 && GpsDataBuf.GPSValidFlag == 1){
         P1.lat = GData.Latitude;
         P1.lon = GData.Longitude;
         P2.lat = GpsDataBuf.Latitude;
         P2.lon = GpsDataBuf.Longitude;
         distance = get_distance(P1, P2);
         LOG_I("GPS distance:%.4f", distance);
         memcpy(&GpsDataBuf, &GData, sizeof(GPS_DATA)); 
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
    Gps.ground_speed = (uint16_t)(strtod(GpsDataBuf.Ground_speed, NULL)*10);
    Gps.hdop = (uint8_t)(strtod(GpsDataBuf.HDOP, NULL)*10);
    Gps.direction = (uint16_t)(strtod(GpsDataBuf.Yaw, NULL)*10);
    Gps.high = (uint16_t)GpsDataBuf.high;
    Gps.Lat = GpsDataBuf.Latitude*1000000;
    Gps.Long = GpsDataBuf.Longitude*1000000;
    #endif
}

void GPS_stop()
{
    MCU_CMD_MARK(CMD_GPS_POWEROFF_INDEX);
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
            NET_CMD_MARK(NET_CMD_Q_LOCATION_D0);
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
        if(Gps.GpsMode == GPS_MODE_TM) {
            if(Gps.GetGPSNum >= GPS_POS_CNT || systm_tick_diff(Gps.Gps_Tm_timeout) > 180*1000) {
                GPS_Composite_PosData();
                NET_CMD_MARK(NET_CMD_Q_LOCATION_D0);
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

