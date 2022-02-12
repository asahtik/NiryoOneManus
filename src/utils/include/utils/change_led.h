#ifdef __arm__
#include <wiringPi.h>
#endif

#ifndef CHANGE_LED_H
#define CHANGE_LED_H

const int LED_GPIO_R = 18;
const int LED_GPIO_G = 24;
const int LED_GPIO_B = 22;

extern void change_led(bool r, bool g, bool b);

#endif