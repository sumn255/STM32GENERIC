#ifndef __MECARUN_V2_MSP_H__
#define __MECARUN_V2_MSP_H__

#ifdef __cplusplus
    extern "C" {
#endif

void TIM1_PWM_Init(void);
void user_Motor_Init(void);
void user_Encoder_Init(void);
void Encoders_Init(void);
void Encoders_DeInit(void);
void Error_Handler(void);

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

#ifdef __cplusplus
    }
#endif

#endif