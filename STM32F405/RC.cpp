#include "label.h"
#include "RC.h"
#include "control.h"
#include "Nuc.h"
#include "Nx.h"

void RC::Decode()
{
	pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	if (sizeof(m_frame) < 18) return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];

	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C 
	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E
}

void RC::OnRC()
{
	if (rc.s[0] != DOWN || rc.s[1] != MID) {
		ctrl.chassis.speedx = rc.ch[0] * para.max_speed / 660.f;
		ctrl.chassis.speedy = -rc.ch[1] * para.max_speed / 660.f;
	}
	//if(ctrl.mode != CONTROL::AUTO)
		ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, rc.ch[3] * para.pitch_speed / 660.f);

	if (rc.s[0] == UP)  //自瞄+导航，不发弹
	{ 
		
		switch (rc.s[1]) {
		case UP:
			ctrl.mode = CONTROL::NORMAL;
			ctrl.pantile.mark_yaw = para.initial_yaw;
			//ctrl.pantile.mark_yaw = para.initial_yaw;
			
			if (nx.targeted) {
				ctrl.chassis.speedx = 0;
				ctrl.chassis.speedy = 0;
			}
			else {
				ctrl.chassis.speedx = -nuc.speedy;
				ctrl.chassis.speedy = nuc.speedx;
			}		
			ctrl.chassis.speedz = 0;// para.rota_speed + sqrt(pow(ctrl.chassis.speedx, 2) + pow(ctrl.chassis.speedy, 2));
						
			ctrl.shooter.openRub = false;
			ctrl.shooter.supply_bullet = false;
			break;
		case MID: //测试自瞄
			ctrl.mode = CONTROL::AUTO;
			//ctrl.supply_motor[1]->setspeed = 2160;// * rc.ch[0]/ 660;
			ctrl.shooter.openRub = true;//开摩擦轮
			if(nx.same_flag < 1000){
				/*if (fabs(nx.yaw_err) < 5) { //误差小于一定值
					//ctrl.shooter.supply_bullet = true;//开供弹轮
				}
				else {
					ctrl.shooter.supply_bullet = false;
				} */
				ctrl.shooter.supply_bullet = true;//开供弹轮
			}
			else
				ctrl.shooter.supply_bullet = false;
			ctrl.chassis.speedz = para.rota_speed + sqrt(pow(ctrl.chassis.speedx, 2) + pow(ctrl.chassis.speedy, 2));
			break;
		case DOWN://完全体
				ctrl.mode = CONTROL::AUTO;// NORMAL;

				ctrl.chassis.speedz = para.rota_speed + sqrt(pow(ctrl.chassis.speedx, 2) + pow(ctrl.chassis.speedy, 2));
			
		
			//ctrl.pantile.mark_yaw = para.initial_yaw;

			if (nx.targeted) {
				ctrl.chassis.speedx = 0;
				ctrl.chassis.speedy = 0;
			}
			else {
				ctrl.chassis.speedx = nuc.speedy;
				ctrl.chassis.speedy = -nuc.speedx;
			}

			ctrl.shooter.openRub = true;//开摩擦轮
			if (nx.same_flag < 1000) {
				/*if (fabs(nx.yaw_err) < 2) { //误差小于一定值
					ctrl.shooter.supply_bullet = true;//开供弹轮
				}
				else {
					ctrl.shooter.supply_bullet = false;
				}*/
				ctrl.shooter.supply_bullet = true;//开供弹轮
			}
			else
				ctrl.shooter.supply_bullet = false;
			break;
		}
	}
	else if (rc.s[0] == MID) //调试行驶
	{
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
		nuc_ctrl = false;
		switch (rc.s[1]) {
		case UP: 
			ctrl.mode = CONTROL::NORMAL;
			ctrl.chassis.speedz = rc.ch[2] * MAXSPEED / -660.f;
			ctrl.pantile.mark_yaw = para.initial_yaw;
			break;
		case MID: 
			ctrl.mode = CONTROL::FOLLOW;
			ctrl.chassis.speedz = 0;
			break;
		case DOWN: 
			ctrl.mode = CONTROL::ROTATION; 
			ctrl.chassis.speedz = para.rota_speed + sqrt(pow(ctrl.chassis.speedx, 2) + pow(ctrl.chassis.speedy, 2));;
			break;
		default: break;
		}
	}
	else if (rc.s[0] == DOWN)  //发射模式
	{
		ctrl.mode = CONTROL::FOLLOW;
		nuc_ctrl = false;
		switch (rc.s[1]) {
		case UP: 
			ctrl.shooter.openRub = true;
			ctrl.shooter.supply_bullet = false;
			break;
		case MID: 
			ctrl.shooter.openRub = true;
			ctrl.shooter.supply_bullet = true;
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.supply_motor[1]->setspeed = rc.ch[0] * 2160 / 660;
			break;
		case DOWN: 
			ctrl.mode = CONTROL::RESET;
			ctrl.shooter.openRub = false;
			ctrl.shooter.supply_bullet = false;
			ctrl.supply_motor[1]->setspeed = rc.ch[0] * 2160 / 660;
			break;
		}
	}

}

void RC::OnPC()
{
	;
}

void RC::Update()
{
	OnRC();
	OnPC();
}


void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}
