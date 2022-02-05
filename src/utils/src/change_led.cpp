#include "utils/change_led.h"

void change_led(bool r, bool g, bool b) {
    wiringPiSetupGpio();

    pinMode(LED_GPIO_R, OUTPUT);
    pinMode(LED_GPIO_G, OUTPUT);
    pinMode(LED_GPIO_B, OUTPUT);

    digitalWrite(LED_GPIO_R, r ? HIGH : LOW);
    digitalWrite(LED_GPIO_G, g ? HIGH : LOW);
    digitalWrite(LED_GPIO_B, b ? HIGH : LOW);
}