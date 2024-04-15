  /*
   *__/\\\_______/\\\__/\\\\____________/\\\\__/\\\________/\\\______________/\\\\\\\\\____________/\\\\\\\\\_____/\\\\\\\\\\\___
   * _\///\\\___/\\\/__\/\\\\\\________/\\\\\\_\/\\\_______\/\\\____________/\\\///////\\\_______/\\\////////____/\\\/////////\\\_
   *  ___\///\\\\\\/____\/\\\//\\\____/\\\//\\\_\/\\\_______\/\\\___________\/\\\_____\/\\\_____/\\\/____________\//\\\______\///__
   *   _____\//\\\\______\/\\\\///\\\/\\\/_\/\\\_\/\\\_______\/\\\___________\/\\\\\\\\\\\/_____/\\\_______________\////\\\_________
   *    ______\/\\\\______\/\\\__\///\\\/___\/\\\_\/\\\_______\/\\\___________\/\\\//////\\\____\/\\\__________________\////\\\______
   *     ______/\\\\\\_____\/\\\____\///_____\/\\\_\/\\\_______\/\\\___________\/\\\____\//\\\___\//\\\____________________\////\\\___
   *      ____/\\\////\\\___\/\\\_____________\/\\\_\//\\\______/\\\____________\/\\\_____\//\\\___\///\\\___________/\\\______\//\\\__
   *       __/\\\/___\///\\\_\/\\\_____________\/\\\__\///\\\\\\\\\/_____________\/\\\______\//\\\____\////\\\\\\\\\_\///\\\\\\\\\\\/___
   *        _\///_______\///__\///_____________\///_____\/////////_______________\///________\///________\/////////____\///////////_____
  */

#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "judgement.h"
#include "led.h"
#include "Nuc.h"
#include "Nx.h"
#include "state.h"

Motor can1_motor[CAN1_MOTOR_NUM] = {
	//12 0.069 0.008
	Motor(M3508,SPD,chassis, ID1, PID(3.5f, 0.f, 0.02f)),//PID(10.f, 0.0655f, 3.49e-4f)),
	Motor(M3508,SPD,chassis, ID2, PID(3.5f, 0.f, 0.02f)),
	Motor(M3508,SPD,chassis, ID3, PID(3.5f, 0.f, 0.02f)),
	Motor(M3508,SPD,chassis, ID4, PID(3.5f, 0.f, 0.02f)),//can1[0]~can1[3]µ×ÅÌµç»ú
	Motor(M6020,POS,chassis,ID5, PID(40.f, 0.01f, 150.f,0.f),PID(1.f, 0.01f, 10.f,0.f)),//33 0.15 9, 0.6 0 0.40
	Motor(M6020,POS,chassis,ID6, PID(40.f, 0.01f, 150.f,0.f),PID(1.f, 0.01f, 10.f,0.f)),
	Motor(M6020,POS,chassis,ID7, PID(40.f, 0.01f, 150.f,0.f),PID(1.f, 0.01f, 10.f,0.f)),
	Motor(M6020,POS,chassis,ID8, PID(40.f, 0.01f, 150.f,0.f),PID(1.f, 0.01f, 10.f,0.f))
};

Motor can2_motor[CAN2_MOTOR_NUM] = {
	Motor(M3508,SPD,shooter, ID1, PID(3.f, 0.f, 0.04f)),
	Motor(M3508,SPD,shooter, ID2, PID(3.f, 0.f, 0.04f)),
	Motor(M3508,SPD,shooter, ID3, PID(3.f, 0.f, 0.04f)),
	Motor(M3508,SPD,shooter, ID4, PID(3.f, 0.f, 0.04f)),
	Motor(M2006,ACE,supply, ID6, PID(0.4f, 0.f, 0.f)),
	Motor(M2006,ACE,supply, ID7, PID(0.1f, 0.f, 1.f)),
	Motor(M6020,POS,pantile, ID5, PID(150.f, 0.04f, 620.f,0.f),
									PID(0.7f, 0.00f, 15.f,0.f)),
	Motor(M6020,POS,pantile, ID8, PID(100.f, 0.04f, 300.f,0.f),
									PID(0.6f, 0.013f, 20.f,0.f))
};

CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_pantile;
DELAY delay;
RC rc;
NUC nuc;
NX nx;
LED led1, led2;
TASK task;
CONTROL ctrl;
Judgement judgement;
PARAMETER para;
State state;


int main(void)
{
	SystemClockConfig();
	delay.Init(168);
	HAL_Init();
	can1.Init(CAN1);
	can2.Init(CAN2);
	timer.Init(BASE, TIM3, 1000).BaseInit();

	judgement.Init(&uart1, 115200, USART1);
	imu_pantile.Init(&uart5, UART5, 115200, CH010);
	rc.Init(&uart2, USART2, 100000);
	nuc.Init(&uart6, USART6, 115200);
	nx.Init(&uart4, UART4, 115200);
	para.Init();
	ctrl.Init(
		{ &can1_motor[0], &can1_motor[1], &can1_motor[2], &can1_motor[3],
		&can1_motor[4], & can1_motor[5],& can1_motor[6],& can1_motor[7], 
		&can2_motor[0], &can2_motor[1], &can2_motor[2], &can2_motor[3],&can2_motor[4],
		& can2_motor[5],&can2_motor[6] ,&can2_motor[7]
		 });
	//led1.Init({ GPIOB,GPIO_PIN_14 });
	//led2.Init({ GPIOB,GPIO_PIN_15 });
	task.Init();
	for (;;)
		;
}





