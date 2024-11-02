#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// float sqrtf(float a) {
//     float x = a;
//     float y = 0;
//     float e = 0.000001; // 设定一个误差范围
//     if (a < 0) {
//   //      printf("Error: Cannot calculate the square root of a negative number.\n");
//         return -1;
//     }
//     while ((x - y) > e || (y - x) > e) {
//         x = (x + a/x) / 2;
//         y = (y + (a/y)) / 2;
//     }
//     return x;
// }


float libtest()
{
	float fb;
    float accel_standard_deviation = sqrtf(10 / 3);
	float pitch = asinf(6);
	float roll = -atan2f(13, 18);
	fb = accel_standard_deviation + pitch + roll;
	return fb;
}