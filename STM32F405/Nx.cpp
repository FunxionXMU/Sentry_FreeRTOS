#include "Nx.h"
#include "CRC.h"
#include "math.h"

void NX::Init(UART* huart, USART_TypeDef* Instance, uint32_t baud)
{
	huart->Init(Instance, baud).DMARxInit(nullptr);
	m_uart = huart;
	m_frame = m_uart->m_uartrx;
	queueHandler = &huart->UartQueueHandler;
}

void NX::Decode()
{
	pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	
	//if (!VerifyCRC16CheckSum(m_frame, 15)) return;

	pre_distance = distance;
	pre_yaw_err = yaw_err;
	pre_pitch_err = pitch_err;

	if(fabs(u8_to_float(m_frame + 1)) < 30)
		pitch_err = u8_to_float(m_frame + 1) + pitch_compensate;
	if(fabs(u8_to_float(m_frame + 5)) < 30)
		yaw_err = u8_to_float(m_frame + 5) + yaw_compensate;
	if(fabs(u8_to_float(m_frame + 9)) < 1000)
		distance = u8_to_float(m_frame + 9) / 100.f; //cm

	if (pre_distance == distance && pre_pitch_err == pitch_err && pre_yaw_err == yaw_err) {
		same_flag++;
	}
	else {
		same_flag = 0;
	}
	if (same_flag > 1000) {
		targeted = false;
	}
	else
		targeted = true;
}