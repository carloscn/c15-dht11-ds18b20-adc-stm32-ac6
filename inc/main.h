/*
 * main.h
 *
 *  Created on: 2018��4��14��
 *      Author: weihaochen
 */

#ifndef MAIN_H_
#define MAIN_H_


/***************** LED���������궨��(�͵�ƽ����) *********************/
#define LED_ON(x)      x=0    //��LED
#define LED_OFF(x)     x=1    //�ر�LED
#define LED_TOGGLE(x)  x^=1   //��תLED
/****************************** end *********************************/



/***************** LED����GPIO��ʱ�Ӻ궨�� ***************************/
#define LED0_GPIO_RCC_CLK  RCC_APB2Periph_GPIOB//LED0 GPIO RCCʱ��
#define LED1_GPIO_RCC_CLK  RCC_APB2Periph_GPIOE//LED1 GPIO RCCʱ��
/****************************** end *********************************/


/********************* LED����GPIO�ں궨�� ***************************/
#define LED0_GPIO  GPIOB
#define LED1_GPIO  GPIOE

#define LED0_PIN   GPIO_Pin_5
#define LED1_PIN   GPIO_Pin_5

#define LED0 PBout(5)// PB5
#define LED1 PEout(5)// PE5

#define LED_GREEN    LED0   //��ɫ
#define LED_BLUE     LED1   //��ɫ
/****************************** end *********************************/


/*************************** ����˵�� *******************************/
//  1������led.hͷ�ļ�
//  2�����ó�ʼ������
//  3��ʹ�ú궨�����LED
//  ʾ��:
//      LED_ON(LED0);//��LED0
//      LED_OFF(LED1);//�ر�LED1
//      LED_TOGGLE(LED_GREEN);//��ת��ɫLED
/****************************** end *********************************/



//��������
#define KEY0  0
#define KEY1  1
#define KEY2  2
#define KEY3  3

//key0 PE4
//key1 PE3
//key2 PE2
//key3/key_up PA0


/********************* ��������GPIO�ں궨�� ***************************/
#define KEY0_GPIO GPIOE
#define KEY1_GPIO GPIOE
#define KEY2_GPIO GPIOE
#define KEY3_GPIO GPIOA

#define KEY0_PIN GPIO_Pin_4
#define KEY1_PIN GPIO_Pin_3
#define KEY2_PIN GPIO_Pin_2
#define KEY3_PIN GPIO_Pin_0
/****************************** end *********************************/


/********************* ��������GPIO��ʱ�Ӻ궨�� *********************/
#define KEY0_RCC_CLK  RCC_APB2Periph_GPIOE
#define KEY1_RCC_CLK  RCC_APB2Periph_GPIOE
#define KEY2_RCC_CLK  RCC_APB2Periph_GPIOE
#define KEY3_RCC_CLK  RCC_APB2Periph_GPIOA
/****************************** end *********************************/



/********************* ����Ӳ��״̬��ȡ�궨�� ***********************/
#define KEY0_HARD_STA  ((KEY0_GPIO->IDR & (uint16_t)KEY0_PIN)==0)
#define KEY1_HARD_STA  ((KEY1_GPIO->IDR & (uint16_t)KEY1_PIN)==0)
#define KEY2_HARD_STA  ((KEY2_GPIO->IDR & (uint16_t)KEY2_PIN)==0)
#define KEY3_HARD_STA  ((KEY3_GPIO->IDR & (uint16_t)KEY3_PIN)!=0)//��������Ǹߵ�ƽ��Ч
/****************************** end *********************************/



/********************* �������·���ֵ�궨�� *************************/
#define KEY0_PRES   1   //KEY0����
#define KEY1_PRES   2   //KEY1����
#define KEY2_PRES   3   //KEY2����
#define WKUP_PRES   4   //KEY_UP����(��WK_UP/KEY_UP)
/****************************** end *********************************/
#include "timer.h"


/********************* ������������ *********************************/

void Key_Init(void);   //IO��ʼ��
u8 Key_Scan(u8);       //����ɨ�躯��
u8 Key_GetSta(u8 key); //������ǰ״̬��ȡ  1:�ѱ�����  0:δ������
/************************* �������� *********************************/
void LED_Init(void);//��ʼ��

void    DHT11_INIT( DHT11 *p_dev );
void    DS18B20_INIT( DS18B20 *dev_p );
void    UART1_INIT( UART* self );
void    UART2_INIT( UART* self );
void    UART3_INIT( UART* self );
void    ADC_INIT( ADC *this );
void    RCC_Configuration(void);
ErrorStatus    system_send_message( void );
void    system_send_all_datas( void );
#endif /* MAIN_H_ */
