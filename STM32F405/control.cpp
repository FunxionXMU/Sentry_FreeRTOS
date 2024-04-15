#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "RC.h"
#include "Nx.h"
#include "SolveTrajectory.h"
#include "Nuc.h"
//#include "imu.h"

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			supply_motor[num4++] = motor[i];
		default:
			break;
		}
	}
	for (int i = 0; i < 4; i++)//舵轮角度初始化
	{
		chassis_motor[i + 4]->setangle = chassis.midhelmangle[i];
	}
	pantile.mark_yaw = para.initial_yaw;
	pantile.mark_pitch = para.initial_pitch;
	pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;

}

void CONTROL::Control_Pantile(float ch_yaw, float ch_pitch)
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//方向相反修改这里正负
	float adjangle = this->pantile.sensitivity * 2;

	switch (ctrl.mode) {
	case NORMAL:
		imu_pantile.preYaw = GetDelta(imu_pantile.GetAngleYaw());//-100
		break;
	case FOLLOW:
		imu_pantile.preYaw = GetDelta(imu_pantile.GetAngleYaw());
		break;
	case AUTO:
		imu_pantile.preYaw -= (adjangle * ch_yaw / 22.75f);
		//if(nuc.reach)
		pantile.Auto_Control();
		pantile.Keep_Pantile(GetDelta(imu_pantile.preYaw), pantile.YAW, imu_pantile);
		break;
	case ROTATION:
		imu_pantile.preYaw -= (adjangle * ch_yaw / 22.75f);
		pantile.Keep_Pantile(GetDelta(imu_pantile.preYaw), pantile.YAW, imu_pantile);		
		break;
	}

	imu_pantile.prePitch = imu_pantile.GetAnglePitch();
	ctrl.pantile.mark_pitch -= (float)(adjangle * ch_pitch);
	if (ctrl.mode != NORMAL) {
		ctrl.pantile.mark_yaw -= (float)(adjangle * ch_yaw);
	}
}

void CONTROL::CHASSIS::Helm_Over_Zero(float* setangle, float curangle)
{
	if (*setangle - curangle > 4096)    //4096 ：半圈机械角度
	{
		*setangle -= 8191;        //8191,8192无所谓了，四舍五入
	}
	else if (*setangle - curangle < -4096)
	{
		*setangle += 8191;
	}
}

void CONTROL::CHASSIS::Keep_Direction()
{
	double s_x = speedx, s_y = speedy;
	double theta;
	/*if (ctrl.mode == AUTO) { //绝对坐标系方向
		theta = ctrl.GetDelta(mechanicalToDegree(ctrl.pantile_motor[PANTILE::TYPE::YAW]->angle[now])
			- mechanicalToDegree(para.initial_yaw) - ctrl.pantile.absolute_yaw) / 180.f * PI;
	}*/
	//else {
		theta = ctrl.GetDelta(mechanicalToDegree(ctrl.pantile_motor[PANTILE::TYPE::YAW]->angle[now])
		- mechanicalToDegree(para.initial_yaw)) / 180.f * PI;
	//}
	
	double st = sin(theta);
	double ct = cos(theta);
	speedx = s_x * ct + s_y * st;
	speedy = -s_x * st + s_y * ct;
}

void CONTROL::CHASSIS::Update()
{
	ctrl.time += 0.001;

	//if(ctrl.mode != NORMAL)
	ctrl.chassis.Keep_Direction();

	if (ctrl.mode == RESET)
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}

	Angle_Change(speedx, speedy, speedz);//舵电机解算
	Speed_Calculate(speedx, speedy, speedz);//轮电机解算

}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference)
{
	float delta = 0, adjust = sensitivity;
	if (type == YAW)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));
		if (delta <= -4096.f)
			delta += 8191.f;
		else if (delta >= 4096.f)
			delta -= 8291.f;
		if (abs(delta) >= 3.f){}
			//mark_yaw += pantile_PID.Delta(delta);
			mark_yaw = ctrl.pantile_motor[0]->angle[0] + pantile_PID.Delta(delta);

	}
	else if (type == PITCH)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;

		if (abs(delta) >= 10.f)
		{
			mark_pitch += delta;
		}
	}
}

void CONTROL::PANTILE::Auto_Control() {
	if (!nx.targeted)
	{
		imu_pantile.preYaw += plus_yaw;
		//mark_pitch += plus_pitch; //扫描目标
		mark_pitch = para.initial_pitch;
		if (mark_pitch >= para.initial_pitch + 100) //pitch到达限位换方向扫描
			plus_pitch = -1 * fabs(plus_pitch);
		if (mark_pitch <= para.pitch_min + 200)
			plus_pitch = fabs(plus_pitch);
	}
	else
	{
		//plus_yaw = fabs(plus_yaw) * ctrl.sgn(rc.yaw_err);  //记录丢失目标的方向，丢失后追踪
		if (!nx.same_flag)//nx数据更新时赋值
		{
			imu_pantile.preYaw += vision_PID[YAW].Position(nx.yaw_err, 2000);

			float theta = (mark_pitch - para.initial_pitch)/ 8192.f * 2 * PI;//枪管的pitch仰角
			st.current_pitch = (theta - nx.pitch_err / 360.f * 2 * PI);//目标与水平线的夹角
			st.xw = nx.distance - st.s_bias;//距离减去二者水平距离（近似认为不变）
			if (st.xw < 0)
				st.xw = 0;
			st.zw = monoDirectionalAirResistanceModel(st.xw, st.current_v, st.current_pitch) - st.h_bias * cos(theta);//弹道补偿,高度减去摄像头和枪口高度差（随theta改变）
			//mark_pitch = pitchTrajectoryCompensation(st.xw, st.zw, st.current_v) / 2 / PI * 8192 + para.initial_pitch;
			mark_pitch -= vision_PID[PITCH].Delta(nx.pitch_err * 8192 / 360);
		}
	}
	imu_pantile.preYaw = ctrl.GetDelta(imu_pantile.preYaw);//限定区间
}

void CONTROL::PANTILE::Update()
{
	absolute_yaw = imu_pantile.GetAngleYaw() - imu_pantile.initYaw + 90;

	if (ctrl.mode == RESET)
	{
		mark_yaw = para.initial_yaw;
		mark_pitch = para.initial_pitch;
	} 

	if (ctrl.mode == AUTO ) { //&& nx.same_flag <= 50
		//mark_yaw -= vision_PID[YAW].Delta(rc.yaw_err);
	}

	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
	if (mark_yaw < 0.0)mark_yaw += 8192.0;
	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min);

	ctrl.pantile_motor[PANTILE::YAW]->setangle = mark_yaw;
	ctrl.pantile_motor[PANTILE::PITCH]->setangle = mark_pitch;
		
}

bool CONTROL::SHOOTER::Judge_Shoot_Heat()
{
	int16_t limit = heat_limit;
	int16_t nowHeat1 = judgement.data.power_heat_data_t.shooter_17mm_1_barrel_heat;
	int16_t nowHeat2 = judgement.data.power_heat_data_t.shooter_17mm_2_barrel_heat;
	int16_t rate = cooling_value;

	if (last_heat == judgement.data.power_heat_data_t.chassis_current)//判断有没有连裁判系统
	{
		shooter_count++;
	}
	else
	{
		shooter_count = 0;
	}
	last_heat = judgement.data.power_heat_data_t.chassis_current;
	if (shooter_count > 500)
	{
		rate = 10;
		nowHeat1 = 0;
		nowHeat1 = 0;
		limit = 50;
	}
	if ((limit - nowHeat1 >= rate / 5.f && limit - nowHeat2 >= rate / 5.f) || heat_ulimit)//为什么乘以2
		return true;
	return false;
}

void CONTROL::SHOOTER::Update()
{
	now_bullet_speed = judgement.data.shoot_data_t.bullet_speed;
	
	if (openRub && Judge_Shoot_Heat())
	{
		ctrl.shooter_motor[0]->setspeed = -shoot_speed;
		ctrl.shooter_motor[1]->setspeed = shoot_speed;
		ctrl.shooter_motor[2]->setspeed = shoot_speed;
		ctrl.shooter_motor[3]->setspeed = -shoot_speed;
	}
	else
	{
		ctrl.shooter_motor[0]->setspeed = 0;
		ctrl.shooter_motor[1]->setspeed = 0;
		ctrl.shooter_motor[2]->setspeed = 0;
		ctrl.shooter_motor[3]->setspeed = 0;
	}

	if (supply_bullet && Judge_Shoot_Heat())
	{
		ctrl.supply_motor[0]->setspeed = supply_speed;
		ctrl.supply_motor[0]->spinning = true;
		ctrl.supply_motor[1]->setspeed = supply_speed;
		ctrl.supply_motor[1]->spinning = true;
	}
	else 
	{
		ctrl.supply_motor[0]->setspeed = 0;
		ctrl.supply_motor[0]->spinning = false;
		ctrl.supply_motor[1]->setspeed = 0;
		ctrl.supply_motor[1]->spinning = false;
	}
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	while (true)
	{
		if (delta <= -180.f)
		{
			delta += 360.f;
		}

		if (delta > 180.f)
		{
			delta -= 360.f;
		}
		if (fabs(delta) <= 180)
			break;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

void CONTROL::CHASSIS::Angle_Change(int32_t speedx, int32_t speedy, int32_t speedz)
{
	if (ctrl.mode == RESET)
	{
		for (int i = 0; i < 4; i++)
		{
			ctrl.chassis_motor[i + 4]->setangle = midhelmangle[i];
		}
		speedz = 0;
		return;
	}
	float theta = atan(1.0 / 1.0);
	float wheel_angle[4];
	float steer_vw = speedz * 0.3 ;

	int32_t avgspeed = (abs(ctrl.chassis_motor[0]->curspeed) + abs(ctrl.chassis_motor[1]->curspeed)
		+ abs(ctrl.chassis_motor[2]->curspeed) + abs(ctrl.chassis_motor[3]->curspeed)) / 4;

	if ((speedx == 0) && (speedy == 0) && (speedz == 0))
	{
		if (avgspeed < 50)
			lock = false;
		if (avgspeed > 1000) //急刹
			lock = true;
		if (!lock)
		{
			for (int i = 0; i < 4; i++)
			{
				wheel_angle[i] = lastSteeringAngletarget[i];
			}
		}
	}
	else
	{
		lock = false;
		wheel_angle[0] = atan2((speedy - steer_vw * sin(theta)),
			(speedx - steer_vw * cos(theta)));       //对应华南理工公式VX=tan-1((vy1-vw1cos45)/(vx1-vw1sin45))

		wheel_angle[1] = atan2((speedy + steer_vw * sin(theta)),
			(speedx - steer_vw * cos(theta)));       //对应华南理工公式VX=tan-1((vy2-vw2cos45)/(vx2+vw2sin45)) 

		wheel_angle[2] = atan2((speedy + steer_vw * sin(theta)),
			(speedx + steer_vw * cos(theta)));       //对应华南理工公式VX=tan-1((vy3+vw3cos45)/(vx3+vw3sin45)) 

		wheel_angle[3] = atan2((speedy - steer_vw * sin(theta)),
			(speedx + steer_vw * cos(theta)));       //对应华南理工公式VX=tan-1((vy4+vw4cos45)/(vx4-vw4sin45)) 
	}

	if (lock) {
		for (int i = 0; i < 4; i++)
		{
			wheel_angle[(i+1)%4] = PI / 4.f * (2 * i - 3);
			//lastSteeringAngletarget[i] = wheel_angle[i];
		}
	}

	for (int i = 0; i < 4; i++)//0~pi
	{
		if (wheel_angle[i] - lastSteeringAngletarget[i] > PI / 2)
		{
			wheel_angle[i] = fmodf(wheel_angle[i] - PI, 2 * PI);
			turnFlag[i] = 1;
		}
		else if (wheel_angle[i] - lastSteeringAngletarget[i] < -PI / 2)
		{
			wheel_angle[i] = fmodf(wheel_angle[i] + PI, 2 * PI);
			turnFlag[i] = 1;
		}
		else
		{
			turnFlag[i] = 0;
		}
	}
	for (int i = 0; i < 4; i++)
	{
		lastSteeringAngletarget[i] = wheel_angle[i];
	}
	int sign = 1;
	for (int i = 0; i < 4; i++)
	{
		if (i % 2 == 0) {
			ctrl.chassis_motor[i + 4]->setangle =  manual_helm(wheel_angle[i] * 4096 / PI  + midhelmangle[i]);
			Helm_Over_Zero(&(ctrl.chassis_motor[i + 4]->setangle), ctrl.chassis_motor[i + 4]->angle[0]);
		}
		else {
			ctrl.chassis_motor[i + 4]->setangle =  manual_helm(wheel_angle[i] * 4096 / PI  + midhelmangle[i]);
			Helm_Over_Zero(&(ctrl.chassis_motor[i + 4]->setangle), ctrl.chassis_motor[i + 4]->angle[0]);

		}
	}

}

void CONTROL::CHASSIS::Speed_Calculate(int32_t speedx, int32_t speedy, int32_t speedz) //轮速解算
{
	float theta = atan(1.0 / 1.0);                     //程序换算角度为弧度不能直接使用！！
	float wheel_rpm[4];

	wheel_rpm[0]
		= -sqrt(pow(speedx - speedz * sin(theta), 2)
			+ pow(speedy - speedz * cos(theta), 2));        //对应华南理工公式  Vx1-Vw1sin45*RADIUS    

	wheel_rpm[1]
		= sqrt(pow(speedx - speedz * sin(theta), 2)
			+ pow(speedy + speedz * cos(theta), 2));      	 //对应华南理工公式  Vx2+Vw2sin45*RADIUS  																																																																								

	wheel_rpm[2]
		= sqrt(pow(speedx + speedz * sin(theta), 2)
			+ pow(speedy + speedz * cos(theta), 2));        //对应华南理工公式  Vy1-Vw1cos45*RADIUS  

	wheel_rpm[3]
		= -sqrt(pow(speedx + speedz * sin(theta), 2)
			+ pow(speedy - speedz * cos(theta), 2));        //对应华南理工公式  Vy2+Vw2cos45*RADIUS 

	double delta_power = power_limit + 100 - judgement.data.power_heat_data_t.chassis_power; //功率控制
	float k = 1;
	if (delta_power < 100)
	{
		k = log2(delta_power / 100.f + 1);
	}
	else
		k = 1;

	if (k < 0.1)
		k = 0.1;

	for (int i = 0; i < 4; i++)
	{

		if (turnFlag[i] == 1)
		{
			wheel_rpm[i] = -wheel_rpm[i];
		}
		else
		{
			wheel_rpm[i] = wheel_rpm[i];
		}
	}
	for (int i = 0; i < 4; i++)
	{
		ctrl.chassis_motor[i]->setspeed = Ramp(wheel_rpm[i], ctrl.chassis_motor[i]->curspeed,200) * k; 
	}
}


float CONTROL::CHASSIS::manual_helm(float sethelm)//-4096~+4096映射到0~8192
{
	float adjangle = this->helm_sensitivity;
	float mark_yaw = (float)(adjangle * (sethelm ));
	//mark_yaw *= (1.f);//方向相反修改这里正负
	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
	if (mark_yaw < 0.0)mark_yaw += 8192.0;
	return mark_yaw;
}

int16_t CONTROL::sgn(float x)
{
	if (x >= 0)
		return 1;
	else
		return -1;
}
