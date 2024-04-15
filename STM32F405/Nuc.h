#pragma once
#include "usart.h."
#include "FreeRTOS.h"
#include <cmath>
#include <cinttypes>
#include "Nx.h"

class NUC
{
public:
	enum { x = 0, y, z };
	float speedx{}, speedy{}, speedz{};
	float ref = 0.1f;
	int32_t positionX = 40, positionY = -15;
	void Decode();
	void Updata();
	void Encode();
	bool reach;		//ÊÇ·ñµ½´ï
	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate);

	uint8_t uarttx[12];
private:
	int tmp[3]{}, flag[3]{};
	QueueHandle_t* queueHandler = NULL;
	BaseType_t pd_Rx{};
	UART* m_uart;
	uint8_t m_frame[UART_MAX_LEN];
};

extern NUC nuc;