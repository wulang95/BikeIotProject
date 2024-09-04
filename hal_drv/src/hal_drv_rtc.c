#include "hal_drv_rtc.h"
#include "ql_api_rtc.h"

#define	leapyear(year)		((year) % 4 == 0)
#define	days_in_year(a)		(leapyear(a) ? 366 : 365)
#define	days_in_month(a)	(month_days[(a) - 1])

#define FEBRUARY		2
#define	STARTOFTIME		1970
#define SECDAY			86400L
#define SECYR			(SECDAY * 365)

/*========================================================================
 *  Global Variable
 *========================================================================*/   
static int month_days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/*========================================================================
 *  function Definition
 *========================================================================*/
static int ql_rtc_fix_weekday(ql_rtc_time_t *rtc_time)
{
    int leapsToDate;
    int lastYear;
    int day;
    int MonthOffset[] = { 0,31,59,90,120,151,181,212,243,273,304,334 };

    if (rtc_time->tm_year < 1753)
        return -1;
    lastYear=rtc_time->tm_year-1;

    //Number of leap corrections to apply up to end of last year
    leapsToDate = lastYear/4 - lastYear/100 + lastYear/400;

    //This year is a leap year if it is divisible by 4 except when it is divisible by 100 unless it is divisible by 400
    //e.g. 1904 was a leap year, 1900 was not, 1996 is, and 2000 will be
    if((rtc_time->tm_year%4==0) && ((rtc_time->tm_year%100!=0) || (rtc_time->tm_year%400==0)) && (rtc_time->tm_mon>2)) 
    {       
        //We are past Feb. 29 in a leap year
        day=1;
    } 
    else 
    {
        day=0;
    }

    day += lastYear*365 + leapsToDate + MonthOffset[rtc_time->tm_mon-1] + rtc_time->tm_mday;

    rtc_time->tm_wday=day%7;

    return 0;
}
static int ql_sec_conv_rtc_time(int64_t* time_t, ql_rtc_time_t *rtc_time)
{
    int    i;
    long   hms, day;

    day = *time_t / SECDAY;
    hms = *time_t % SECDAY;

    //Hours, minutes, seconds are easy
    rtc_time->tm_hour = hms / 3600;
    rtc_time->tm_min = (hms % 3600) / 60;
    rtc_time->tm_sec = (hms % 3600) % 60;

    //Number of years in days
    for (i = STARTOFTIME; day >= days_in_year(i); i++) 
    {
        day -= days_in_year(i);
    }
    rtc_time->tm_year = i;

    //Number of months in days left
    if (leapyear(rtc_time->tm_year)) 
    {
        days_in_month(FEBRUARY) = 29;
    }
    for (i = 1; day >= days_in_month(i); i++) 
    {
        day -= days_in_month(i);
    }
    days_in_month(FEBRUARY) = 28;
    rtc_time->tm_mon = i;

    //Days are what is left over (+1) from all that.
    rtc_time->tm_mday = day + 1;

    //Determine the day of week
    return ql_rtc_fix_weekday(rtc_time);
}

static int ql_rtc_conv_sec_time(int64_t* time_t, ql_rtc_time_t* rtc_time)
{
    int mon = rtc_time->tm_mon;
    int year = rtc_time->tm_year;
    int64_t days, hours;

    mon -= 2;
    if (0 >= (int)mon) 
    {    
        // 1..12 -> 11,12,1..10
        mon += 12;    
        
        //Puts Feb last since it has leap day
        year -= 1;
    }
    days = (unsigned long)(year / 4 - year / 100 + year / 400 +367 * mon / 12 + rtc_time->tm_mday) + year * 365 - 719499;

    hours = (days * 24) + rtc_time->tm_hour;

    *time_t = (hours * 60 + rtc_time->tm_min) * 60 + rtc_time->tm_sec;

    return 0;
}

void hal_drv_rtc_set_time(int64_t timestamp)
{
    ql_rtc_time_t tm;
    ql_sec_conv_rtc_time(&timestamp, &tm);
    ql_rtc_set_time(&tm);
}


int64_t hal_drv_rtc_get_timestamp()
{
    ql_rtc_time_t tm;
    int64_t time_t;
    ql_rtc_get_time(&tm);
    ql_rtc_conv_sec_time(&time_t, &tm);
    return time_t;
}

void hal_drv_rtc_set_alarm(int64_t sec, void(*rtc_alarm_call)())
{
    int64_t time_t;
    ql_rtc_time_t tm;
    time_t = hal_drv_rtc_get_timestamp();
    time_t += sec;
    ql_sec_conv_rtc_time(&time_t, &tm);
    ql_rtc_set_alarm(&tm);
    ql_rtc_register_cb(rtc_alarm_call);
    ql_rtc_enable_alarm(1);
}

void hal_drv_rtc_time_print()
{
    ql_rtc_time_t tm;
    ql_rtc_get_localtime(&tm);
    ql_rtc_print_time(tm);
}



