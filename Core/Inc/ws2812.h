//#ifndef WS2812_H
//#define WS2812_H
//
//#include "stm32h7xx_hal.h"

//// Number of LEDs
//#define WS2812_LED_COUNT 4
//
//// TIM & DMA Configuration
//#define WS2812_TIM        htim1      // Change based on the timer used
//#define WS2812_DMA        hdma_tim1_ch1 // Change based on the DMA channel
//#define WS2812_GPIO_PORT  GPIOE
//#define WS2812_GPIO_PIN   GPIO_PIN_9  // Change based on your setup
//
//// WS2812 Color Struct
//typedef struct {
//    uint8_t red;
//    uint8_t green;
//    uint8_t blue;
//} WS2812_Color;
//
//// Function Prototypes
//void WS2812_Init(void);
//void WS2812_SetColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
//void WS2812_Update(void);
/*
 * ws2812.h
 *
 *  Created on: 21 de jul. de 2024
 *      Author: FelipeAlmeida
 */

#ifndef WS2812_H
#define WS2812_H

#include "stm32f4xx_hal.h"
//#define WS2812_T0H_CYCLES 170   // Adjust for 800kHz (High Pulse ~0.85µs)
//#define WS2812_T1H_CYCLES  180   // Adjust for 800kHz (Low Pulse ~0.4µs)
#define WS2812_T0H_CYCLES  (29)   // Adjust based on your timer settings
#define WS2812_T1H_CYCLES  (58)   // Adjust based on your timer settings
#define NUM_LEDS 4




void ws2812_init(TIM_HandleTypeDef *htim_ptr, DMA_HandleTypeDef *hdma_ptr, uint32_t tim_channel);
void ws2812_set_colors(TIM_HandleTypeDef *htim_ptr, uint8_t *colors, int num_leds);
//void ws2812_set_colors(uint8_t *colors, int num_leds);
void update_single_led(int led_index, uint8_t red, uint8_t green, uint8_t blue);

#endif // WS2812_H

//#endif
