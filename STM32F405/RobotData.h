#pragma once

class RobotEvent
{
	//��ϸ�����뷭�ġ�RoboMaster 2021 ����ϵͳ����Э�鸽¼ V1.6��20231122����
public:
	struct {
		uint8_t Sof;	//1 ��ʼ�ֽڣ��̶�0xA5
		uint16_t data_length;  //2 ���ݳ���
		uint8_t seq;	//3 �����
		uint8_t CRC8;	//4 ֡ͷУ��
	}Rxheader;

	//0x0001 ����״̬���ݣ� Ƶ�ʣ�1hz  ȫ������� ������· 
	struct {
		uint8_t game_type : 4;       //1 ��������
		uint8_t game_progress : 4;   //2 ��ǰ�����׶�
		uint16_t stage_remain_time;  //3 ʣ��ʱ��
		uint64_t SyncTimeStamp;		 //4 UNIXʱ��
	}game_status_t;

	//0x0002 �����������  Ƶ�ʣ���������  ȫ������� ������·
	struct {
		uint8_t winner;  // 0ƽ�� 1��ʤ 2��ʤ
	}game_result_t;

	//0x0003 ������Ѫ������  Ƶ�ʣ�3hz   ȫ�������  ������·
	//���û�����δ�ϳ����߱����£���Ѫ��Ϊ 0
	struct {
		uint16_t red_1_robot_HP;  //1 �� 1 Ӣ�ۻ�����Ѫ����
		uint16_t red_2_robot_HP;  //2 �� 2 ���̻�����Ѫ��
		uint16_t red_3_robot_HP;  //3 �� 3 ����������Ѫ��
		uint16_t red_4_robot_HP;  //4 �� 4 ����������Ѫ��
		uint16_t red_5_robot_HP;  //5 �� 5 ����������Ѫ��
		uint16_t red_7_robot_HP;  //6 �� 7 �ڱ�������Ѫ��
		uint16_t red_outpost_HP;  //7 �췽ǰ��վѪ��
		uint16_t red_base_HP;     //8 �췽����Ѫ��
		uint16_t blue_1_robot_HP; //9 �� 1 Ӣ�ۻ�����Ѫ��
		uint16_t blue_2_robot_HP; //10 �� 2 ���̻�����Ѫ��
		uint16_t blue_3_robot_HP; //11 �� 3 ����������Ѫ��
		uint16_t blue_4_robot_HP; //12 �� 4 ����������Ѫ��
		uint16_t blue_5_robot_HP; //13 �� 5 ����������Ѫ��
		uint16_t blue_7_robot_HP; //14 �� 7 �ڱ�������Ѫ��
		uint16_t blue_outpost_HP; //15 ����ǰ��վѪ��
		uint16_t blue_base_HP;    //16 ��������Ѫ��
	}game_robot_HP_t;

	//0x0101  ���������¼����� Ƶ�ʣ�1hz  ����ȫ�������  ������·
	struct {
		uint32_t event_data; //����վ���������أ��ߵأ����⻤�ܣ��з����ڻ���ʱ��
	}event_data_t;

	//0x0102  ����վ������ʶ����  Ƶ�ʣ�����վ�����ͷ�ʱ��������  ����ȫ�������  ������·
	struct {
		uint8_t reserved;				 //1 ����
		uint8_t supply_robot_id;		 //2 ����������ID
		uint8_t supply_projectile_step;  //3 �����ڿ���״̬
		uint8_t supply_projectile_num;   //4 ��������
	}ext_supply_projectile_action_t;

	//0x0104  ���о�������  Ƶ�ʣ������з�/�и�ʱ���ͣ�����1hz  ��������ȫ�������  ������·
	struct {
		uint8_t level;					//1 �з��ȼ�
		uint8_t offending_robot_id;		//2 �з�������ID
		uint8_t count;					//3 �з�������Υ�����
	}referee_warning_t;

	//0x0105  ���ڷ����������  Ƶ�ʣ�1hz  ����ȫ�������  ������·
	struct {
		uint8_t dart_remaining_time;	//1 �������ڷ���ʣ��ʱ�䣬��λ����
		uint8_t dart_aim_state;			//2 ����Ŀ���������
	}dart_info_t;

	//0x0201 ������������ϵ����  Ƶ�ʣ�10hz  ��Ӧ������ ������·
	struct {
		uint8_t robot_id;							  //1 ��������ID
		uint16_t current_exp;						  //2 �����˵�ǰ����ֵ
		uint16_t current_HP;						  //3 �����˵�ǰѪ��
		uint8_t power_management_gimbal_output : 1;	  //4 gimbal�ڵ�Դ���
		uint8_t power_management_chassis_output : 1;  //5 chassis�ڵ�Դ���
		uint8_t power_management_shooter_output : 1;  //6 shooter�ڵ�Դ���
	}robot_status_t;

	//0x0202  ʵʱ������������  Ƶ�ʣ�50hz  ��Ӧ������ ������·
	struct {
		uint16_t chassis_voltage;				//1 ��Դ����ģ�� chassis �������ѹ����λ��mV��
		uint16_t chassis_current;				//2 ��Դ����ģ�� chassis �������������λ��mA��
		float chassis_power;					//3 ���̹��ʣ���λ��W��
		uint16_t buffer_energy;					//4 ������������λ��J��
		uint16_t shooter_17mm_1_barrel_heat;	//5 �� 1 �� 17mm ���������ǹ������
		uint16_t shooter_17mm_2_barrel_heat;	//6 �� 2 �����������ǹ������
		uint16_t shooter_42mm_barrel_heat;		//7 42mm ���������ǹ������
	}power_heat_data_t;

	//0x0203 ������λ������  Ƶ�ʣ��ڱ�10hz������1hz  ��Ӧ������  ������·
	struct {
		float x;		//1 ��������λ�� x ���꣬��λ��m
		float y;		//2 ��������λ�� y ���꣬��λ��m
		float angle;	//3 �������˲���ģ�鳯�򣬵�λ���ȡ�����Ϊ 0 ��
	}robot_pos_t;

	//0x0204  ��������������  Ƶ�ʣ�3hz  ��Ӧ������  ������·
	struct {
		uint8_t recovery_buff;	//1 �����˻�Ѫ���棨�ٷֱȣ�ֵΪ 10 ��Ϊÿ��ظ� 10%���Ѫ����
		uint8_t cooling_buff;	//2 ������ǹ����ȴ���ʣ�ֱ��ֵ��ֵΪ 5 ��ζ�� 5 ����ȴ��
		uint8_t defence_buff;	//3 �����˷������棨�ٷֱȣ�ֵΪ 50 ��Ϊ 50%�������棩
		uint16_t attack_buff;	//4 �����˹������棨�ٷֱȣ�ֵΪ 50 ��Ϊ 50%�������棩
	}buff_t;

	//0x0205  ����֧Ԯʱ������ Ƶ�ʣ�1hz �������л����� ������·
	struct {
		uint8_t airforce_status;	//1 ���л�����״̬��0 Ϊ������ȴ��1 Ϊ��ȴ��ϣ�2 Ϊ����֧Ԯ�ڼ䣩
		uint8_t time_remain;		//2 ��״̬��ʣ��ʱ�䣨��λΪ s������ȡ��)
	}air_support_data_t;

	//0x0206  �˺�״̬����    Ƶ�ʣ��˺���������  ��Ӧ������  ������·
	struct {
		uint8_t armor_id : 4;				//1 װ��ģ������ģ��ID��ţ�������ֵΪ0�����鿴�ֲ�
		uint8_t HP_deduction_reason : 4;	//2 Ѫ���仯����
	}hurt_data_t;

	//0x0207 ʵʱ�������    Ƶ�ʣ����跢�����  ��Ӧ������  ������·
	struct {
		uint8_t bullet_type;			//1 ��������
		uint8_t shooter_number;			//2 ������� ID
		uint8_t launching_frequency;	//3 �������٣���λ��Hz��
		float initial_speed;			//4 ������ٶȣ���λ��m/s��
	}shoot_data_t;

	//0x0208  ��������  Ƶ�ʣ�10hz  ���������������л�����   ������·
	struct {
		uint16_t projectile_allowance_17mm;		//1 17mm ������������
		uint16_t projectile_allowance_42mm;		//2 42mm ������������
		uint16_t remaining_gold_coin;			//3 ʣ��������
	}projectile_allowance_t;

	//0x0209  ������ RFID ״̬  Ƶ�ʣ�3hz  ����������  ������·
	struct {
		uint32_t rfid_status;	//���ز�ͬ�ط��Ƿ��Ѽ�⵽������� RFID
	}rfid_status_t;

	//0x020A  ����ѡ�ֶ�ָ������  Ƶ�ʣ�բ�����ߺ�3hz  ��������  ������·
	struct {
		uint8_t dart_launch_opening_status;		//1 ��ǰ���ڷ���վ��״̬
		uint8_t dart_attack_target;				//2 ���ڵĴ��Ŀ�꣺��Ĭ��Ϊǰ��վ��
		uint16_t target_change_time;			//3 �л����Ŀ��ʱ�ı���ʣ��ʱ�䣬��λ��s����δ�л�����Ĭ��Ϊ 0
		uint16_t latest_launch_cmd_time;		//4 ���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ��s����ʼֵΪ 0
	}dart_client_cmd_t;

	//0x020B  ���������λ������  Ƶ�ʣ�1hz  �����ڱ�������  ������·
	struct {
		float hero_x;			//1 ����Ӣ�ۻ�����λ�� x �����꣬��λ��m
		float hero_y;			//2 ����Ӣ�ۻ�����λ�� y �����꣬��λ��m
		float engineer_x;		//3 �������̻�����λ�� x �����꣬��λ��m
		float engineer_y;		//4 �������̻�����λ�� y �����꣬��λ��m
		float standard_3_x;		//5 ���� 3 �Ų���������λ�� x �����꣬��λ��m
		float standard_3_y;		//6 ���� 3 �Ų���������λ�� y �����꣬��λ��m
		float standard_4_x;		//7 ���� 4 �Ų���������λ�� x �����꣬��λ��m
		float standard_4_y;		//8 ���� 4 �Ų���������λ�� y �����꣬��λ��m
		float standard_5_x;		//9 ���� 5 �Ų���������λ�� x �����꣬��λ��m
		float standard_5_y;		//10 ���� 5 �Ų���������λ�� y �����꣬��λ��m
	}ground_robot_position_t;

	//0x020C �״��ǽ������� Ƶ�ʣ�1hz  �����״������  ������·
	struct {
		uint8_t mark_hero_progress;			//1 �Է�Ӣ�ۻ����˱���ǽ��ȣ�0��120
		uint8_t mark_engineer_progress;		//2 �Է����̻����˱���ǽ��ȣ�0��120
		uint8_t mark_standard_3_progress;	//3 �Է� 3 �Ų��������˱���ǽ��ȣ�0��120
		uint8_t mark_standard_4_progress;	//4 �Է� 4 �Ų��������˱���ǽ��ȣ�0��120
		uint8_t mark_standard_5_progress;	//5 �Է� 5 �Ų��������˱���ǽ��ȣ�0��120
		uint8_t mark_sentry_progress;		//6 �Է��ڱ������˱���ǽ��ȣ�0��120
	}radar_mark_data_t;

	//0x020D �ڱ��������������Ϣͬ�� Ƶ�ʣ�1hz  �����ڱ������� ������·
	struct {
		uint32_t sentry_info;  //�һ����������һ�Ѫ������
	}sentry_info_t;

	//0x020E �״�����������Ϣͬ��  Ƶ�ʣ�1hz  �����״������  ������·
	struct {
		uint8_t radar_info;
	}radar_info_t;

	//0x0301  �����˽�������  Ƶ�ʣ����ͷ��������ͣ�����10hz  ������·
	struct {
		uint16_t data_cmd_id;	//1 ������ ID,��Ϊ���ŵ������� ID
		uint16_t sender_id;		//2 ������ ID,�������� ID ƥ�䣬ID ��������¼
		uint16_t receiver_id;	//3 ������ ID,ID ��������¼
		uint8_t* user_data;	    //4 x ���Ϊ 113
	}robot_interaction_data_t;

	//0x0303  ѡ�ֶ�С��ͼ��������  Ƶ�ʣ���̨�ִ�������  ������·
	struct {
		float target_position_x;	//1 Ŀ��λ�� x �����꣬��λ m,������Ŀ������� ID ʱ����ֵΪ 0
		float target_position_y;	//2 Ŀ��λ�� y �����꣬��λ m,������Ŀ������� ID ʱ����ֵΪ 0
		uint8_t cmd_keyboard;		//3 ��̨�ְ��µļ��̰���ͨ�ü�ֵ,�ް���Ϊ0
		uint8_t target_robot_id;	//4 �Է������� ID
		uint16_t cmd_source;			//5 ��Ϣ��Դ ID������¼
	}map_command_t;
};

//0x0305  ѡ�ֶ�С��ͼ�����״�����  Ƶ�ʣ�����10hz  ������·
struct map_robot_data_t {
	uint16_t target_robot_id;	//1 Ŀ������� ID
	float target_position_x;	//2 Ŀ�� x λ�����꣬��λ��m����������ʾ
	float target_position_y;	//3 Ŀ�� x λ�����꣬��λ��m����������ʾ
};

//0x0307 ѡ�ֶ�С��ͼ�����ڱ�����  Ƶ�ʣ�1hz ������·
struct map_data_t {
	uint8_t intention;			//1 �ƶ���Ŀ��㣬���Թ������߷���
	uint16_t start_position_x;	//2 ·����� x �����꣬��λ��dm
	uint16_t start_position_y;	//3 ·����� y �����꣬��λ��dm
	int8_t delta_x[49];			//4 ·���� x ���������飬��λ��dm
	int8_t delta_y[49];			//5 ·���� y ���������飬��λ��dm
	uint16_t sender_id;			//6 ������ ID���������� ID ƥ�䣬ID ��������¼
};

//0x0308 ѡ�ֶ�С��ͼ���ջ�������Ϣ 3hz   ������·
struct custom_info_t {
	uint16_t sender_id;
	uint16_t receiver_id;
	uint16_t user_data[30];
};

//ͼ����·���ݣ��Զ��������

//0x0302 �Զ��������������˽�������  <30Hz
struct custom_robot_data_t {
	uint8_t data[20];
};

//0x0304 ����ң������ 30hz
//��ͨ��ң�������͵ļ���ң������ͬ��ͨ��ͼ����·���͸���Ӧ�����ˡ�
struct remote_control_t {
	int16_t mouse_x;			//��� x ���ƶ��ٶȣ���ֵ��ʶ�����ƶ�
	int16_t mouse_y;			//��� y ���ƶ��ٶȣ���ֵ��ʶ�����ƶ�
	int16_t mouse_z;			//�������ƶ��ٶȣ���ֵ��ʶ������
	int8_t left_button_down;	//�������Ƿ��£�0 Ϊû���£�1 Ϊ����
	int8_t right_button_down;	//����Ҽ��Ƿ��£�0 Ϊû���£�1 Ϊ����
	uint16_t keyboard_value;
	uint16_t reserved;
};


//0x0301�Ľ�����������
//		|
//		|
//		v


//0x0100 ѡ�ֶ�ɾ��ͼ��
struct interaction_layer_delete_t {
	uint8_t delete_type;	//ɾ������
	uint8_t layer;			//ͼ����
};

//0x0101 ѡ�ֶ˻���һ��ͼ��
//0x0102  ѡ�ֶ˻��ƶ���ͼ��
//0x0103 ѡ�ֶ˻������ͼ��
//0x0104  ѡ�ֶ˻����߸�ͼ��
struct interaction_figure_t {
	uint8_t figure_name[3];		//ͼ����,��ͼ��ɾ�����޸ĵȲ����У���Ϊ����
	uint32_t operate_tpye : 3;	//ͼ�β���
	uint32_t figure_tpye : 3;	//ͼ������
	uint32_t layer : 4;			//ͼ���� 0-9
	uint32_t color : 4;			//��ɫ
	uint32_t details_a : 9;		//�����ƵĲ�ͬͼ�κ��岻ͬ
	uint32_t details_b : 9;		//�����ƵĲ�ͬͼ�κ��岻ͬ
	uint32_t width : 10;		//�߿����������С���߿����Ϊ 10��1
	uint32_t start_x : 11;		//���/Բ�� x ����
	uint32_t start_y : 11;		//���/Բ�� y ����
	uint32_t details_c : 10;	//�����ƵĲ�ͬͼ�κ��岻ͬ
	uint32_t details_d : 11;	//�����ƵĲ�ͬͼ�κ��岻ͬ
	uint32_t details_e : 11;	//�����ƵĲ�ͬͼ�κ��岻ͬ
};



//0x0110 ѡ�ֶ˻����ַ�ͼ��
struct ext_client_custom_character_t {
	interaction_figure_t grapic_data_struct;
	uint8_t data[30];
};

//0x0120 �ڱ���������ָ��
struct sentry_cmd_t {
	uint32_t sentry_cmd;
};

//0x0121 �״���������ָ��
struct radar_cmd_t {
	uint8_t radar_cmd;
};