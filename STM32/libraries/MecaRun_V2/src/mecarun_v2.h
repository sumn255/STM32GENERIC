#ifndef MECARUN_V2_h
#define MECARUN_V2_h
#include "Arduino.h"

#define DRVTIM TIM1
#define DUTY_MAX 1000

#define MT_P_BANK GPIOA
#define MT_N_BANK GPIOB

#define MT_1P_PIN LL_GPIO_PIN_8
#define MT_2P_PIN LL_GPIO_PIN_9
#define MT_3P_PIN LL_GPIO_PIN_10
#define MT_4P_PIN LL_GPIO_PIN_11

#define MT_1N_PIN LL_GPIO_PIN_13
#define MT_2N_PIN LL_GPIO_PIN_14
#define MT_3N_PIN LL_GPIO_PIN_15
#define MT_4N_PIN LL_GPIO_PIN_12

//#define RBTH 100

typedef struct{
	int32_t target;
	int32_t cnt;
	int32_t cnt_last;
	float err_last;
	float err_int;
	float kp;
	float ki;
	float kd;
	int32_t outlimit;
	float intlimit;
}pidtype;

typedef struct{
	uint8_t motor_en;
	uint8_t motor_update;
	uint8_t pid_en;
	uint8_t pid_update_period;
}mt_ctrltype;

typedef struct{
	uint32_t time;
	int16_t x;
	int16_t y;
	int16_t r;
}speed3axistype;


class Mecarun_v2
{
	public:
		Mecarun_v2(void);//pwm out init
		void Move(int16_t *speedarr);
		void PID_Enable(float kp,float ki,float kd);//encoder init,pid argcs init
		void PID_Disable(void);
		void Get_cnt(int32_t *cntarr);
		void Sync_cnt(void);
		void Set_speed(void);
		void Set_speed_bypass(int16_t *speedarr);//for test
		void cal_mecanum(void);
		void Get_speed(speed3axistype *speedarr);

	protected:
		//void Set_PWM_ag(void);//for open loop
		//void Set_PWM_he(void);//for close loop
		void pid_Motor_Init(void);
		pidtype motor[4];
		mt_ctrltype mt_ctrl;
		speed3axistype speed_xyr;
};

#endif
