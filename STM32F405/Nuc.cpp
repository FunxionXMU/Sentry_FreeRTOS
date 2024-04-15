#include "Nuc.h"
#include "label.h"
#include "string.h"


void NUC::Decode()
{
	tmp[x] = tmp[y] = 0;
	pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	memcpy(m_frame, m_uart->m_uartrx, 100);
	if (m_frame[0] == 'A' && m_frame[14] == 'a')
	{
		for (int i = 1; i <= 13; i++)
			m_frame[i] -= '0';
		tmp[y] = m_frame[2] * 10000 + m_frame[3] * 1000 + m_frame[4] * 100 + m_frame[5] * 10 + m_frame[6];
		tmp[x] = m_frame[8] * 10000 + m_frame[9] * 1000 + m_frame[10] * 100 + m_frame[11] * 10 + m_frame[12];

		if (m_frame[1] == 1) tmp[y] *= -1;
		if (m_frame[7] == 1) tmp[x] *= -1;

		if (reach) {
			speedx = 0;
			speedy = 0;
		}
		else {
			speedx = tmp[x] * ref;
			speedy = tmp[y] * ref;
		}

		if (m_frame[13]) {
			reach = true;
		}
			
		else if (nx.same_flag > 1000)
			reach = false;
		//reach = (m_frame[13] == 1) ? true : false;
	}
}

void NUC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void NUC::Encode()
{
	/*uarttx[0] = (uint8_t)'D';
	uarttx[1] = (uint8_t)'1';
	uarttx[2] = (uint8_t)'A';
	uarttx[3] = (uint8_t)'B';
	uarttx[4] = (uint8_t)'C';
	uarttx[5] = (uint8_t)'0';
	uarttx[6] = (uint8_t)'A';
	uarttx[7] = (uint8_t)'B';
	uarttx[8] = (uint8_t)'C';
	uarttx[9] = (uint8_t)'H';*/
	m_uart->UARTTransmit(uarttx, 12);
}