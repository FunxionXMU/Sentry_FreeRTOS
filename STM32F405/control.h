#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

class CONTROL final
{
public:
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};

	double time = 0;
	float g = 9.7887f;
	
	enum MODE { ROTATION, RESET, NORMAL, FOLLOW, AUTO, SHOOT} mode;
	struct CHASSIS
	{
		bool turnFlag[4];
		float lastSteeringAngletarget[4];
		const float helm_sensitivity = 1.f;
		float midhelmangle[4] = { 5943 + 2048,130 + 2048,5940 + 2048,3808 + 2048 };// 3619 2934 3592 2917
		int32_t count, last_current;
		int32_t now_Hp = 0;
		bool openBuffer = false;
		bool lock = false; //急刹标志
		uint16_t power_limit = 100;

		PID chassis_reset{};
		PID speed_limit_PID{ 0.1f,0.02f,0.02f };
		int32_t speedx{}, speedy{}, speedz{};

		float manual_helm(float ch_yaw);
		void Helm_Over_Zero(float* setangle, float curangle);
		void Speed_Calculate(int32_t speedx, int32_t speedy, int32_t speedz); //轮速解算
		void Angle_Change(int32_t speedx, int32_t speedy, int32_t speedz);
		void Keep_Direction();

		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	private:
		double del_used_power_limit = 0.0;
	};

	struct PANTILE
	{
		enum TYPE { YAW, PITCH };
		float mark_pitch = 6200, mark_yaw = 270;
		float vision_mark_pitch = 6200, vision_mark_yaw = 270;
		PID pantile_PID = {3.f,0.01f,0.5f};//0.1 0 -1; 0.38 0 -4.6; 0.5 0 -5.5
		PID vision_PID[2] = { {0.1f,0.0f,0.f},{0.2f,0.0f,0.f} };//
		const float sensitivity = 0.7f;
		bool aim = false;
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference);
		float plus_pitch = 0.3f, plus_yaw = 0.07f; //自动模式云台扫描速率
		float absolute_yaw;

		void Update();
		void Auto_Control();
	};

	struct SHOOTER
	{

		float now_bullet_speed = 0.f;

		bool auto_shoot = false;
		bool openRub = false, supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;
		int16_t shoot_speed = 6200;
		int16_t supply_speed = 4000;//2160;
		uint16_t heat_limit = 400;
		uint16_t cooling_value = 80;

		void Update();
		bool Judge_Shoot_Heat();

	private:
		int32_t shooter_count, last_heat;
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;
	
	static int16_t Setrange(const int16_t original, const int16_t range);
	void Control_Pantile(float ch_yaw, float ch_pitch);
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
	int16_t sgn(float x);
	
private:

};

extern CONTROL ctrl;