#pragma once
#include "stm32f4xx_hal.h"
#include "judgement.h"
#include "Nuc.h"

//enum Pos_state { JiDi, Depot, SightAim1, SightAim2, Attack };//���أ����������棬��׼����������
enum Pos_state { Defense, Depot, Gain };	//�����������������
enum { Red, Blue };
class State
{
public:
	uint8_t game_type;			//��������
	uint8_t game_progress;		//�����׶�
	uint16_t game_countdown;	//����ʣ���ʱ
	uint16_t NowHp, PreHp;		//��ǰ����һ��Ѫ��
	uint8_t hurt_armor_id;		//��Ѫid
	uint8_t hurt_reason;		//Ѫ���仯����
	uint16_t total_Recover_Hp;	//�ۼƲ�Ѫ
	float Yaw_delta_defense;	//����ת��Ƕ�ֵ

	uint16_t Hp1, Hp3;			//1�ţ�3��Ѫ��

	//char Pos[5][11] = { {"1007710198"},{"0024610431"},{"1028000190"},{"1007410393"},{"1028000190"} };
	char Pos[3][11] = { {"0000310396"},{"0022410420"},{"1028810082"} };
	Pos_state destination;		//Ŀ��

	uint8_t RobotID;
	uint32_t time_count = 300;
	uint32_t initcount = 0;

	bool pantile_state; //0��ʾ������1��ʾ����
	bool invincible = true;	//�޵�״̬��1��ʾ�޵У�0��ʾʧЧ
	uint8_t OwnSide;

	void Update();
	void manual_state();
	void shield_update(); //�����޵�״ֵ̬
	void Init();
};

extern State state;