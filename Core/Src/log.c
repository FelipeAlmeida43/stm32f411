/*
 * log.c
 *
 *  Created on: Feb 18, 2025
 *      Author: FelipeAlmeida
 */


/*
 * log.c
 *
 *  Created on: Feb 7, 2025
 *      Author: fs141194
 */
#include "stdio.h"
#include "stdarg.h"
#include "stm32f4xx_hal.h"
#include "log.h"
#define COLOR_RESET   "\x1b[0m"
#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_BLUE    "\x1b[34m"
#define COLOR_MAGENTA "\x1b[35m"
#define COLOR_CYAN    "\x1b[36m"

extern enum LogLevel level;

void Log(UART_HandleTypeDef *huart,LogLevel level, const char *format, ...) {
    va_list args;
    va_start(args, format);

    switch (level) {
        case LOG_LEVEL_INFO:
        	//HAL_UART_Transmit(huart, COLOR_GREEN, sizeof(COLOR_GREEN), 100);
            printf(COLOR_GREEN);
            printf("[INFO]: ");
            break;
        case LOG_LEVEL_WARNING:
            printf(COLOR_YELLOW);
            printf("[WARNING]: ");
            break;
        case LOG_LEVEL_VERBOSE:
            printf(COLOR_CYAN);
            printf("[VERBOSE]: ");
            break;
        case LOG_LEVEL_ERROR:
             printf(COLOR_RED);
             printf("[ERROR]: ");
             break;
        default:
            printf(COLOR_RESET);
            break;
    }

    vprintf(format, args);
    printf(COLOR_RESET);
    printf("\n");

    va_end(args);
}


