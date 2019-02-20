//*****************************************************************************
//
// mecarun_v2.cpp - Drives for the timers to drive the mecanum wheels.
//
// Copyright (c) 2016-2019 Elecholic.tech.  All rights reserved.
// Software License Agreement
//
// This software is under BSD license.
//
//*****************************************************************************

#include "Arduino.h"
#include "mecarun_v2.h"
#include "mecarun_v2_msp.h"
#include "ll_libs.h"
#include "config.h"

/*Encoder for motor 1*/
static int16_t cnt1 = 0;
/*end Encoder for motor 1*/

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

Mecarun_v2::Mecarun_v2(void)
{
	TIM1_PWM_Init();
	user_Motor_Init();
	this->mt_ctrl.motor_en = 1;
}


void Mecarun_v2::Move(int16_t *speedarr)
{
	this->speed_xyr.x = speedarr[1];
	this->speed_xyr.y = speedarr[0];
	this->speed_xyr.r = speedarr[2];
	this->cal_mecanum();
	if(this->mt_ctrl.pid_en == 0)
	{
		this->mt_ctrl.motor_update = 1;
		this->Set_speed();
	}
}

void Mecarun_v2::PID_Enable(float kp,float ki,float kd)
{
	uint8_t i;
	for(i=0;i<4;++i)
	{
		this->motor[i].cnt_last = this->motor[i].cnt;
		this->motor[i].err_int = 0;
		this->motor[i].err_last = 0;
		this->motor[i].kp = kp;
		this->motor[i].ki = ki;
		this->motor[i].kd = kd;
		this->motor[i].outlimit = 1000;
		this->motor[i].intlimit = 20000;
	}
	this->mt_ctrl.pid_update_period = 2;
	this->mt_ctrl.pid_en = 1;
	Encoders_Init();
	pid_Motor_Init();
	cnt1 = 0;
	user_Encoder_Init();
}

void Mecarun_v2::PID_Disable(void)
{
	Encoders_DeInit();
	this->mt_ctrl.pid_en = 0;
	cnt1 = 0;
}

void Mecarun_v2::Sync_cnt(void)
{
	#if ENCODER_REVERSE == 0
	this->motor[0].cnt += (int32_t)cnt1;
	this->motor[2].cnt += (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim3)));
	this->motor[1].cnt += (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim4)));
	this->motor[3].cnt += (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim5)));
	#else
	this->motor[0].cnt -= (int32_t)cnt1;
	this->motor[2].cnt -= (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim3)));
	this->motor[1].cnt -= (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim4)));
	this->motor[3].cnt -= (int32_t)((int16_t)(__HAL_TIM_GET_COUNTER(&htim5)));
	#endif

	cnt1 = 0;
	LL_TIM_SetCounter(htim3.Instance, 0);
	LL_TIM_SetCounter(htim4.Instance, 0);
	LL_TIM_SetCounter(htim5.Instance, 0);
}

void Mecarun_v2::Set_speed(void)
{
	static uint32_t time_next = 0;
	uint32_t time_curr;
	int32_t  speed[4];
	float currerr[4] = {0,0,0,0};
	uint8_t i;

	if(this->mt_ctrl.motor_en == 0)//motors disable
		return;
	if(this->mt_ctrl.pid_en == 0 )//no pid
	{
		if(this->mt_ctrl.motor_update == 0)
			return;
		for(i=0;i<4;++i)//open loop
			#if MOTOR_REVERSE == 0
			speed[i] = this->motor[i].target;
			#else
			speed[i] = -this->motor[i].target;
			#endif

		//set pwm
		if(speed[3] >= 0)//keep decay mode the same
		{
			if(speed[0] >= 0)//0 ch1
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1N,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_ResetOutputPin(MT_P_BANK, MT_1P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_ALTERNATE);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_ResetOutputPin(MT_N_BANK, MT_1N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_OUTPUT);
				speed[0] = -speed[0];
			}
			speed[0] = (speed[0] > DUTY_MAX ? DUTY_MAX:speed[0]);
			LL_TIM_OC_SetCompareCH1(DRVTIM,(uint32_t)speed[0]);

			if(speed[1] >= 0)//1 ch2
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_ResetOutputPin(MT_N_BANK, MT_2N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_OUTPUT);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2N,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_ResetOutputPin(MT_P_BANK, MT_2P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_ALTERNATE);
				speed[1] = -speed[1];
			}
			speed[1] = (speed[1] > DUTY_MAX ? DUTY_MAX:speed[1]);
			LL_TIM_OC_SetCompareCH2(DRVTIM,(uint32_t)speed[1]);

			if(speed[2] >= 0)//2 ch3
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3N,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_ResetOutputPin(MT_P_BANK, MT_3P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_ALTERNATE);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_ResetOutputPin(MT_N_BANK, MT_3N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_OUTPUT);
				speed[2] = -speed[2];
			}
			speed[2] = (speed[2] > DUTY_MAX ? DUTY_MAX:speed[2]);
			LL_TIM_OC_SetCompareCH3(DRVTIM,(uint32_t)speed[2]);

			//3 ch4
			LL_GPIO_ResetOutputPin(MT_N_BANK, MT_4N_PIN);
			LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
			speed[3] = (speed[3] > DUTY_MAX ? DUTY_MAX:speed[3]);
			LL_TIM_OC_SetCompareCH4(DRVTIM,(uint32_t)speed[3]);

		}

		else
		{
			if(speed[0] >= 0)//0 ch1
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_SetOutputPin(MT_N_BANK, MT_1N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_OUTPUT);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1N,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_SetOutputPin(MT_P_BANK, MT_1P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_ALTERNATE);
				speed[0] = -speed[0];
			}
			speed[0] = (speed[0] > DUTY_MAX ? DUTY_MAX:speed[0]);
			LL_TIM_OC_SetCompareCH1(DRVTIM,(uint32_t)speed[0]);

			if(speed[1] >= 0)//1 ch2
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2N,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_SetOutputPin(MT_P_BANK, MT_2P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_ALTERNATE);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_SetOutputPin(MT_N_BANK, MT_2N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_OUTPUT);
				speed[1] = -speed[1];
			}
			speed[1] = (speed[1] > DUTY_MAX ? DUTY_MAX:speed[1]);
			LL_TIM_OC_SetCompareCH2(DRVTIM,(uint32_t)speed[1]);

			if(speed[2] >= 0)//2 ch3
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_LOW);
				LL_GPIO_SetOutputPin(MT_N_BANK, MT_3N_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_ALTERNATE);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_OUTPUT);
			}
			else
			{
				LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3N,LL_TIM_OCPOLARITY_HIGH);
				LL_GPIO_SetOutputPin(MT_P_BANK, MT_3P_PIN);
				LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_OUTPUT);
				LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_ALTERNATE);
				speed[2] = -speed[2];
			}
			speed[2] = (speed[2] > DUTY_MAX ? DUTY_MAX:speed[2]);
			LL_TIM_OC_SetCompareCH3(DRVTIM,(uint32_t)speed[2]);

			//3 ch4
			LL_GPIO_SetOutputPin(MT_N_BANK, MT_4N_PIN);
			LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_LOW);
			speed[3] = (speed[3] > DUTY_MAX ? DUTY_MAX:speed[3]);
			LL_TIM_OC_SetCompareCH4(DRVTIM,(uint32_t)-speed[3]);
		}
		this->mt_ctrl.motor_update = 0;


	}
	else//pid mode
	{
		time_curr = HAL_GetTick();
		if(time_curr >= time_next)
		{
			this->Sync_cnt();
			for(i=0;i<4;++i)
			{
				//currspeedwatch[i]=(float)(mt[i].cnt-mt[i].cnt_last)/(time_curr-time_next+ctrl->pid_update_period)*1000.0/3.08;
				currerr[i]=(float)this->motor[i].target*3.08-(float)(this->motor[i].cnt-this->motor[i].cnt_last)/(time_curr-time_next+this->mt_ctrl.pid_update_period)*1000.0; //3080pps
				this->motor[i].err_int += currerr[i];
				this->motor[i].err_int = (this->motor[i].err_int > this->motor[i].intlimit)?this->motor[i].intlimit:this->motor[i].err_int;
				this->motor[i].err_int = (this->motor[i].err_int < -this->motor[i].intlimit)?(-this->motor[i].intlimit):this->motor[i].err_int;
				this->motor[i].cnt_last = this->motor[i].cnt;

				#if MOTOR_REVERSE == 0
				//speedwatch[i]=speed[i]=currerr[i]*(this->motor[i].kp)+this->motor[i].err_int*(this->motor[i].ki)+(currerr[i]-this->motor[i].err_last)*(this->motor[i].kd);
				speed[i]=currerr[i]*(this->motor[i].kp)+this->motor[i].err_int*(this->motor[i].ki)+(currerr[i]-this->motor[i].err_last)*(this->motor[i].kd);

				#else
				//speedwatch[i]=speed[i]=-(currerr[i]*(mt[i].kp)+mt[i].err_int*(mt[i].ki)+(currerr[i]-mt[i].err_last)*(mt[i].kd));
				speed[i]=-(currerr[i]*(this->motor[i].kp)+this->motor[i].err_int*(this->motor[i].ki)+(currerr[i]-this->motor[i].err_last)*(this->motor[i].kd));
				#endif

				this->motor[i].err_last = currerr[i];
				speed[i] = (speed[i] > this->motor[i].outlimit)?this->motor[i].outlimit:speed[i];
				speed[i] = (speed[i] < -this->motor[i].outlimit)?(-this->motor[i].outlimit):speed[i];
			}
			time_next = time_curr + this->mt_ctrl.pid_update_period;

			//set pwm
			{
				if(speed[0] >= 0)//0 ch1
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_OUTPUT);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_ALTERNATE);
				}
				else
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_ALTERNATE);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_OUTPUT);
					speed[0] = -speed[0];
				}
				speed[0] = (speed[0] > DUTY_MAX ? DUTY_MAX:speed[0]);
				LL_TIM_OC_SetCompareCH1(DRVTIM,(uint32_t)speed[0]);

				if(speed[1] >= 0)//1 ch2
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_ALTERNATE);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_OUTPUT);
				}
				else
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_OUTPUT);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_ALTERNATE);
					speed[1] = -speed[1];
				}
				speed[1] = (speed[1] > DUTY_MAX ? DUTY_MAX:speed[1]);
				LL_TIM_OC_SetCompareCH2(DRVTIM,(uint32_t)speed[1]);

				if(speed[2] >= 0)//2 ch3
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_OUTPUT);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_ALTERNATE);
				}
				else
				{
					LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_ALTERNATE);
					LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_OUTPUT);
					speed[2] = -speed[2];
				}
				speed[2] = (speed[2] > DUTY_MAX ? DUTY_MAX:speed[2]);
				LL_TIM_OC_SetCompareCH3(DRVTIM,(uint32_t)speed[2]);

				if(speed[3] >= 0)//3 ch4
				{
					LL_GPIO_ResetOutputPin(MT_N_BANK, MT_4N_PIN);
					LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
				}
				else
				{
					LL_GPIO_SetOutputPin(MT_N_BANK, MT_4N_PIN);
					LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_LOW);
					speed[3] = -speed[3];
				}
				speed[3] = (speed[3] > DUTY_MAX ? DUTY_MAX:speed[3]);
				LL_TIM_OC_SetCompareCH4(DRVTIM,(uint32_t)speed[3]);
			}
		}
	}
}

void Mecarun_v2::Set_speed_bypass(int16_t *speedarr)//for test
{
	uint8_t i;
	for(i=0;i<4;++i)
	{
		this->motor[i].target = speedarr[i];
	}
	if(this->mt_ctrl.pid_en == 0)
	{
		this->mt_ctrl.motor_update = 1;
		this->Set_speed();
	}
}

void Mecarun_v2::Get_cnt(int32_t *cntarr)
{
	uint8_t i;
	for(i=0;i<4;++i)
		cntarr[i]=this->motor[i].cnt;
}

/*Encoder for motor 1*/
extern "C" void EXTI2_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) ==
				HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))
			++cnt1;
		else
			--cnt1;
	}
}

extern "C" void EXTI15_10_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) == 
				HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))
			--cnt1;
		else
			++cnt1;
	}
}
/*end Encoder for motor 1*/

void Mecarun_v2::pid_Motor_Init(void)
{
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH4);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_EnableChannel(DRVTIM, LL_TIM_CHANNEL_CH3N);

	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH4,LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH1N,LL_TIM_OCPOLARITY_LOW);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH2N,LL_TIM_OCPOLARITY_LOW);
	LL_TIM_OC_SetPolarity(DRVTIM, LL_TIM_CHANNEL_CH3N,LL_TIM_OCPOLARITY_LOW);

	LL_GPIO_ResetOutputPin(MT_P_BANK, MT_1P_PIN);
	LL_GPIO_ResetOutputPin(MT_P_BANK, MT_2P_PIN);
	LL_GPIO_ResetOutputPin(MT_P_BANK, MT_3P_PIN);
	LL_GPIO_ResetOutputPin(MT_P_BANK, MT_4P_PIN);

	LL_GPIO_ResetOutputPin(MT_N_BANK, MT_1N_PIN);
	LL_GPIO_ResetOutputPin(MT_N_BANK, MT_2N_PIN);
	LL_GPIO_ResetOutputPin(MT_N_BANK, MT_3N_PIN);
	LL_GPIO_ResetOutputPin(MT_N_BANK, MT_4N_PIN);

	LL_GPIO_SetPinMode(MT_P_BANK, MT_1P_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_2P_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_3P_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_P_BANK, MT_4P_PIN, LL_GPIO_MODE_ALTERNATE);

	LL_GPIO_SetPinMode(MT_N_BANK, MT_1N_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_N_BANK, MT_2N_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_N_BANK, MT_3N_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(MT_N_BANK, MT_4N_PIN, LL_GPIO_MODE_OUTPUT);
}
