#include "Key.h"

uint8_t Key1_GetNum(void)
{
    uint8_t KeyNum = 0;
    if (DL_GPIO_readPins(KEY_KEY1_PORT, KEY_KEY1_PIN)>>7 == 0)
    {
        delay_ms(20);
        while (DL_GPIO_readPins(KEY_KEY1_PORT, KEY_KEY1_PIN)>>7 == 0);
        delay_ms(20);
        KeyNum = 1;
    }

    return KeyNum;
}

uint8_t Key2_GetNum(void)
{
    uint8_t KeyNum = 0;
    if (DL_GPIO_readPins(KEY_KEY2_PORT, KEY_KEY2_PIN)>>18 == 0)
    {
        delay_ms(20);
        while (DL_GPIO_readPins(KEY_KEY2_PORT, KEY_KEY2_PIN)>>18 == 0);
        delay_ms(20);
        KeyNum = 1;
    }

    return KeyNum;
}

uint8_t Key3_GetNum(void)
{
    uint8_t KeyNum = 0;
    if (DL_GPIO_readPins(KEY_KEY3_PORT, KEY_KEY3_PIN)>>19 == 0)
    {
        delay_ms(20);
        while (DL_GPIO_readPins(KEY_KEY3_PORT, KEY_KEY3_PIN)>>19 == 0);
        delay_ms(20);
        KeyNum = 1;
    }

    return KeyNum;
}

uint8_t Key4_GetNum(void)
{
    uint8_t KeyNum = 0;
    if (DL_GPIO_readPins(KEY_KEY4_PORT, KEY_KEY4_PIN)>>20 == 0)
    {
        delay_ms(20);
        while (DL_GPIO_readPins(KEY_KEY4_PORT, KEY_KEY4_PIN)>>20 == 0);
        delay_ms(20);
        KeyNum = 1;
    }

    return KeyNum;
}

uint8_t Key5_GetNum(void)
{
    uint8_t KeyNum = 0;
    if (DL_GPIO_readPins(KEY_KEY5_PORT, KEY_KEY5_PIN)>24 == 0)
    {
        delay_ms(20);
        while (DL_GPIO_readPins(KEY_KEY5_PORT, KEY_KEY5_PIN)>>24 == 0);
        delay_ms(20);
        KeyNum = 1;
    }

    return KeyNum;
}

uint8_t Key6_GetNum(void)
{
    uint8_t KeyNum = 0;
    if (DL_GPIO_readPins(KEY_KEY6_PORT, KEY_KEY6_PIN)>>27 == 0)
    {
        delay_ms(20);
        while (DL_GPIO_readPins(KEY_KEY6_PORT, KEY_KEY6_PIN)>>27 == 0);
        delay_ms(20);
        KeyNum = 1;
    }

    return KeyNum;
}