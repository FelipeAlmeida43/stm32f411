/*
 * ws2812.c
 *
 *  Created on: Feb 14, 2025
 *      Author: fs141194
 */
#include "ws2812.h"


//#define WS2812_PWM_HIGH 170   // Adjust for 800kHz (High Pulse ~0.85µs)
//#define WS2812_PWM_LOW  180   // Adjust for 800kHz (Low Pulse ~0.4µs)
//#define WS2812_RESET_PULSES 25 // Reset timing (50µs)
//
//extern TIM_HandleTypeDef WS2812_TIM;
//extern DMA_HandleTypeDef WS2812_DMA;
//
//static WS2812_Color led_buffer[WS2812_LED_COUNT];
//static uint16_t pwm_buffer[WS2812_LED_COUNT * 24 + WS2812_RESET_PULSES]; // Data + Reset
//
//// Convert RGB values to PWM pulses
//static void WS2812_ConvertToPWM(void) {
//    uint16_t idx = 0;
//
//    for (uint8_t led = 0; led < WS2812_LED_COUNT; led++) {
//        uint32_t color = ((led_buffer[led].green << 16) |
//                          (led_buffer[led].red << 8)  |
//                          (led_buffer[led].blue));
//
//        for (int8_t bit = 23; bit >= 0; bit--) {
//            pwm_buffer[idx++] = (color & (1 << bit)) ? WS2812_PWM_HIGH : WS2812_PWM_LOW;
//        }
//    }
//
//    // Append RESET signal (low pulses)
//    for (uint8_t i = 0; i < WS2812_RESET_PULSES; i++) {
//        pwm_buffer[idx++] = 0;
//    }
//}
//
//// Initialize WS2812 LEDs
//void WS2812_Init(void) {
//
//    HAL_TIM_PWM_Start_DMA(&WS2812_TIM, TIM_CHANNEL_1, (uint32_t *)pwm_buffer, sizeof(pwm_buffer)/sizeof(uint16_t));
//}
//
//// Set color for individual LED
//void WS2812_SetColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue) {
//    if (index >= WS2812_LED_COUNT) return;
//    led_buffer[index].red = red;
//    led_buffer[index].green = green;
//    led_buffer[index].blue = blue;
//}
//
//// Update LED strip
//void WS2812_Update(void) {
//
//    WS2812_ConvertToPWM();
//    HAL_TIM_PWM_Start_DMA(&WS2812_TIM, TIM_CHANNEL_1, (uint32_t *)pwm_buffer, sizeof(pwm_buffer)/sizeof(uint16_t));
//
//
//}
//
//// DMA Transfer Complete Callback (optional)
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
//    if (htim == &WS2812_TIM) {
//        HAL_TIM_PWM_Stop_DMA(&WS2812_TIM, TIM_CHANNEL_1);
//    }
//}
/*
 * ws2812.c
 *
 *  Created on: 21 de jul. de 2024
 *      Author: FelipeAlmeida
 */
#include "stm32f4xx_hal.h"
#include "ws2812.h"
#include "log.h"

#define LED_BITS 24
#define RESET_SLOTS 50
#define TOTAL_BITS (NUM_LEDS * LED_BITS + RESET_SLOTS)

static uint16_t ws2812_buffer[TOTAL_BITS];
uint8_t ws2812_colors[NUM_LEDS * 3];
static TIM_HandleTypeDef htim;
static DMA_HandleTypeDef hdma;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
volatile uint8_t ws2812_dma_done = 0;  // Global flag to track DMA completion
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {  // Check if it's TIM3 (or your chosen timer)
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);  // Stop PWM after transmission
        ws2812_dma_done = 1;  // Set a flag to indicate DMA is done
    }
}
void ws2812_init(TIM_HandleTypeDef *htim_ptr, DMA_HandleTypeDef *hdma_ptr, uint32_t tim_channel) {
    htim = *htim_ptr;
    hdma = *hdma_ptr;

    if(HAL_TIM_PWM_Init(&htim1)!= HAL_OK){
    	Log(&huart2, LOG_LEVEL_ERROR,"HAL_TIM_PWM_Init_FAILED\r\n");
    }else Log(&huart2, LOG_LEVEL_INFO,"HAL_TIM_PWM_Init_SUCCESS\r\n");
}



void ws2812_set_colors(TIM_HandleTypeDef *htim_ptr, uint8_t *colors, int num_leds) {
    int bit_index = 0;

    for (int led = 0; led < num_leds; led++) {
        uint32_t color = (colors[led * 3] << 16) | (colors[led * 3 + 1] << 8) | colors[led * 3 + 2];
        for (int i = 23; i >= 0; i--) {
            ws2812_buffer[bit_index++] = (color & (1 << i)) ? WS2812_T1H_CYCLES : WS2812_T0H_CYCLES;
        }
    }

    // Adicionar código de reset
    for (int i = 0; i < RESET_SLOTS; i++) {
        ws2812_buffer[bit_index++] = 0;
    }
    htim = *htim_ptr;

    ws2812_dma_done = 0;
    if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)ws2812_buffer, TOTAL_BITS) != HAL_OK) {
        Log(&huart2, LOG_LEVEL_ERROR, "HAL_TIM_PWM_Start_DMA_FAILED\r\n");
    } else {
        Log(&huart2, LOG_LEVEL_INFO, "HAL_TIM_PWM_Start_DMA_SUCCESS\r\n");
    }
}
void update_single_led(int led_index, uint8_t green, uint8_t red, uint8_t blue) {
    if (led_index < 0 || led_index >= NUM_LEDS) return;

    // Atualizar a cor do LED específico no array global
    ws2812_colors[led_index * 3] = green;
    ws2812_colors[led_index * 3 + 1] = red;
    ws2812_colors[led_index * 3 + 2] = blue;

    // Atualizar os LEDs com as novas cores
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
    ws2812_set_colors(&htim, ws2812_colors, NUM_LEDS);
}
