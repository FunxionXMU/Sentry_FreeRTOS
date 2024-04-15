#pragma once
#include <stm32f4xx_hal.h>
#include "usart.h"
#include "label.h"
#include "FreeRTOS.h"
#include "string.h"

class NX
{
public:
	uint8_t rx_len;
	uint8_t recv_end_flag;
	void Init(UART* huart, USART_TypeDef* Instance, uint32_t baud);
	void Decode();

	float yaw_err{}, pitch_err{}, distance{}, pre_yaw_err{}, pre_pitch_err{}, pre_distance{};
	float yaw_compensate = 0.f, pitch_compensate = 10.f;
	uint16_t same_flag = 0;
	bool targeted = false;
	BaseType_t pd_Rx{};
	QueueHandle_t* queueHandler = NULL;

private:
	UART* m_uart;
	uint8_t* m_frame;
	float u8_to_float(uint8_t* p) { float s{}; memcpy(&s, p, 4); return s; }
};

extern NX nx;