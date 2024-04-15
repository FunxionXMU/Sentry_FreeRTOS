#include "state.h"
#include "Nuc.h"

//void State::manual_state()
//{
//	if (time_count > 264) {
//		destination = SightAim1;
//
//	}
//	else {
//		destination = JiDi;
//	}
//
//	//if (!invincible) {
//	//	destination = JiDi;		//无敌失效后进入基地防御模式
//	//}
//
//	if (NowHp <= 300) {
//		if (total_Recover_Hp < 550)
//			destination = Depot;
//	}
//
//	nuc.uarttx[0] = (uint8_t)'D';
//	memcpy(nuc.uarttx + 1, &Pos[destination], 10);
//	nuc.uarttx[11] = (uint8_t)'H';
//}

void State::manual_state()
{
	
	if (time_count > 0 && time_count <= 300&&NowHp>=500) {
		destination = Defense;
	}

	if (NowHp <= 300&&destination!=Depot) {
		if (total_Recover_Hp <= 500)
			destination = Depot;

		//if(destination==Depot)
	}
	else if (NowHp >= 600 && destination == Depot)
	{
		destination = Defense;
	}

	

	nuc.uarttx[0] = (uint8_t)'D';
	memcpy(nuc.uarttx + 1, &Pos[destination], 10);
	nuc.uarttx[11] = (uint8_t)'H';
}

void State::shield_update()	//判断己方无敌状态是否失效
{
	if (Hp1 <= 0 && Hp3 <= 0) {
		invincible = false;		//己方无敌状态失效
	}
}

void State::Update()
{
	game_type = judgement.data.game_status_t.game_type;
	game_progress = judgement.data.game_status_t.game_progress;
	game_countdown = judgement.data.game_status_t.stage_remain_time;
	PreHp = NowHp;
	NowHp = judgement.data.robot_status_t.current_HP;
	if ((NowHp - PreHp) > 0)		//计算总回血量
		total_Recover_Hp += (NowHp - PreHp);
	hurt_armor_id = judgement.data.hurt_data_t.armor_id;
	hurt_reason = judgement.data.hurt_data_t.HP_deduction_reason;
	RobotID = judgement.data.robot_status_t.robot_id;	//获取id
	OwnSide = (RobotID <= 11) ? Red : Blue;		//判断己方的颜色
	if (OwnSide == Red) {
		Hp1 = judgement.data.game_robot_HP_t.red_1_robot_HP;
		Hp3= judgement.data.game_robot_HP_t.red_3_robot_HP;
	}
	else {
		Hp1 = judgement.data.game_robot_HP_t.blue_1_robot_HP;
		Hp3 = judgement.data.game_robot_HP_t.blue_3_robot_HP;
	}

	if (game_type == 4 && game_progress == 4) {  //当比赛为高校联盟赛且比赛进行中
		time_count = game_countdown;
	}
	if (game_type == 4 && game_progress == 3)	//进入5秒倒计时
	{
		total_Recover_Hp = 0;
		invincible = true;
	}
}

void State::Init()
{
	destination = Depot;
	time_count = 0;

}