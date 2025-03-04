#include "label.h"
#include "taskslist.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "Nuc.h"
#include "Nx.h"
#include "judgement.h"
#include "state.h"

void TASK::Init()
{
	//创建开始任务
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度
}

/*
开始任务任务函数
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建任务

	xTaskCreate((TaskFunction_t)LedTask,
		(const char*)"LedTask",
		(uint16_t)LED_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)LED_TASK_PRIO,
		(TaskHandle_t*)&LedTask_Handler);

	xTaskCreate((TaskFunction_t)DecodeTask,
		(const char*)"DecodeTask",
		(uint16_t)DECODE_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)DECODE_TASK_PRIO,
		(TaskHandle_t*)&DecodeTask_Handler);

	xTaskCreate((TaskFunction_t)MotorUpdateTask,
		(const char*)"MotorUpdateTask",
		(uint16_t)MOTOR_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)MOTOR_TASK_PRIO,
		(TaskHandle_t*)&MotorTask_Handler);

	xTaskCreate((TaskFunction_t)CanTransimtTask,
		(const char*)"CanTransimtTask",
		(uint16_t)CANTX_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CANTX_TASK_PRIO,
		(TaskHandle_t*)&CanTxTask_Handler);

	xTaskCreate((TaskFunction_t)ControlTask,
		(const char*)"ControlTask",
		(uint16_t)CONTROL_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CONTROL_TASK_PRIO,
		(TaskHandle_t*)&ControlTask_Handler);

	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}

void MotorUpdateTask(void* pvParameters)
{
	while (1)
	{
		for (auto& motor : can1_motor)motor.Ontimer(can1.data, can1.temp_data);
		for (auto& motor : can2_motor)motor.Ontimer(can2.data, can2.temp_data);
		vTaskDelay(4);
	}
}

void CanTransimtTask(void* pvParameters)
{
	while (true)
	{
		//judgement.BuffData();
		switch (timer.counter % 5)
		{
		case 0:
			can1.Transmit(0x200, can1.temp_data);
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8);
			break;
		case 2:
			can2.Transmit(0x200, can2.temp_data);
			break;
		case 3:
			can2.Transmit(0x1ff, can2.temp_data + 8);
			break;
		}
		vTaskDelay(4);
	}
}

void ControlTask(void* pvParameters)
{
	while (true)
	{
		ctrl.chassis.Update();
		ctrl.pantile.Update();
		ctrl.shooter.Update();
		rc.Update();
		state.Update();
		judgement.BuffData();
		vTaskDelay(1);
	}
}


void DecodeTask(void* pvParameters)
{
	while (true)
	{
		rc.Decode();
		imu_pantile.Decode();
		state.manual_state();
		nuc.Decode();
		nx.Decode();
		judgement.GetData();
		vTaskDelay(4);
	}
}

void LedTask(void* pvParameters)
{
	while (true)
	{
		/*if (task.led_flag)
		{
			led1.on();
			led2.off();
			task.led_flag = false;
		}
		else
		{
			led1.off();
			led2.on();
			task.led_flag = true;
		}*/
		vTaskDelay(500);
	}
}





