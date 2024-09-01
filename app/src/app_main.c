/*app_main入口*/
#include "sys_core.h"



void app_main()
{
    sys_init();         /*外设驱动初始化   系统参数初始化 */
    car_init();         /*  对整车初始化  */

    def_rtos_task_create()

}