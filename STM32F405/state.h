#pragma once
#include "stm32f4xx_hal.h"
#include "judgement.h"
#include "Nuc.h"

//enum Pos_state { JiDi, Depot, SightAim1, SightAim2, Attack };//基地，补给，增益，瞄准，攻击敌人
enum Pos_state { Defense, Depot, Gain };	//防御，补给，增益点
enum { Red, Blue };
class State
{
public:
	uint8_t game_type;			//比赛类型
	uint8_t game_progress;		//比赛阶段
	uint16_t game_countdown;	//比赛剩余计时
	uint16_t NowHp, PreHp;		//当前和上一次血量
	uint8_t hurt_armor_id;		//扣血id
	uint8_t hurt_reason;		//血量变化类型
	uint16_t total_Recover_Hp;	//累计补血
	float Yaw_delta_defense;	//击打转变角度值

	uint16_t Hp1, Hp3;			//1号，3号血量

	//char Pos[5][11] = { {"1007710198"},{"0024610431"},{"1028000190"},{"1007410393"},{"1028000190"} };
	char Pos[3][11] = { {"0000310396"},{"0022410420"},{"1028810082"} };
	Pos_state destination;		//目标

	uint8_t RobotID;
	uint32_t time_count = 300;
	uint32_t initcount = 0;

	bool pantile_state; //0表示导航，1表示自瞄
	bool invincible = true;	//无敌状态，1表示无敌，0表示失效
	uint8_t OwnSide;

	void Update();
	void manual_state();
	void shield_update(); //更新无敌状态值
	void Init();
};

extern State state;