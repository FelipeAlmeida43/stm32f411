/*
 * log.h
 *
 *  Created on: Feb 7, 2025
 *      Author: fs141194
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

typedef enum {
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_VERBOSE,
	LOG_LEVEL_ERROR,
	LOG_LEVEL_DEBUG,
} LogLevel;
void Log(UART_HandleTypeDef *huart,LogLevel level, const char *format, ...);
#endif /* INC_LOG_H_ */
