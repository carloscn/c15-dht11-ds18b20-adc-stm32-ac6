/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */


#include "stm32f10x.h"
#include "global.h"
#include "main.h"

u8 Key_Sta[4]={0};//按键的状态
UART      uart_a, *uart_a_handle;
UART      uart_b, *uart_b_handle;
UART      uart_c, *uart_c_handle;
DHT11       dht11, *dht11_handle;
DS18B20     ds18b20_a, *ds18b20_a_handle;
ADC         adc, *adc_handle;
float       temp_max = 30.0f;
uint16      gsm_freeze_count = 0;

BYTE*   at_sms_config   = "AT+CMGF=1";
BYTE*   at_sms_setgsm   = "AT+CSCS=\"GSM\"";
BYTE*   at_sms_number   = "AT+CMGS=\"18300041052\"";
BYTE*   at_sms_content  = "warning, the system warning!";
BYTE    at_sms_start    = 0x1A;
BYTE*   at_enter        = "\r\n";
int i;


bool    allow_send_flag = false;
u32     timCnt=0;
bool    allow_time_flag = false;


#define INTERVAL_TIME  (1000)      //定时时间,单位ms

int main(void)
{
    float   temp;
    BYTE * temp_src[20];
    BYTE * adc_str[10];


    delay_init();
    uart_a_handle = &uart_a;
    uart_b_handle = &uart_b;
    uart_c_handle = &uart_c;
    ds18b20_a_handle = &ds18b20_a;
    dht11_handle   =   &dht11;
    adc_handle      =   &adc;
    RCC_Configuration();
    LED_Init();
    Key_Init();
    UART1_INIT( uart_a_handle );
    UART2_INIT( uart_b_handle );
    UART3_INIT( uart_c_handle );
    GPIO_ResetBits(LED0_GPIO,LED0_PIN);
    DS18B20_INIT( ds18b20_a_handle );
    DHT11_INIT(dht11_handle);
    ADC_INIT( adc_handle );
    TIM_SetInterval(1,50000);//50ms
    //for( i = 0; i < 8; i ++)
        DELAY_MS(1000);
    GPIO_ResetBits(LED1_GPIO,LED1_PIN);
    for(;;) {
#if 0
        uart_c_handle->write_string(uart_c_handle, "temp : ",-1);
        ds18b20_a_handle->save_temp_string( ds18b20_a_handle, temp_src );
        uart_c_handle->write_string(uart_c_handle, temp_src ,7);
        uart_c_handle->write_string(uart_c_handle, "\r\n" ,-1);

        uart_c_handle->write_string(uart_c_handle, "humi : ",-1);
        dht11_handle->get_all_value( dht11_handle );
        uart_c_handle->write_string(uart_c_handle, dht11_handle->humi_str ,7);
        uart_c_handle->write_string(uart_c_handle, " % \r\n",-1);
#endif
        switch(Key_Scan(0)){
            case KEY0_PRES:{
                if(Key_GetSta(KEY2)){//按键同时按下测试,按住KEY2 再按KEY0


                    printf("KEY0 Press!");
                }
                else{
                    printf("KEY2 & KEY0 Press!");
                    LED_TOGGLE(LED0);
                }
                break;
            }
            case KEY1_PRES:{
                printf("KEY1 Press!");
                allow_send_flag = false;
                LED_TOGGLE(LED1);
                break;
            }
            case KEY2_PRES:{
                allow_send_flag = true;
                printf("KEY2 Press!");
                LED_TOGGLE(LED0);
                LED_TOGGLE(LED1);
                break;
            }
            case WKUP_PRES:{//关闭所有LED
                printf("KEY3 Press!");
                LED_OFF(LED0);
                LED_OFF(LED1);
                break;
            }
        }
        if( allow_send_flag == true && allow_time_flag == true) {
            system_send_all_datas();
            allow_time_flag = false;
        }

    }
}

void TIM1_IRQHandler(void)   //TIM中断
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET){  //检查TIM更新中断发生与否
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update  );  //清除TIMx更新中断标志
        //LED1=!LED1;
        timCnt ++;
        if( timCnt >= 20  ) {
            allow_time_flag = true;
            timCnt = 0;
        }
    }
}

// 12    3  4     5  6     7  8      9  10    11 12  1314
// @@  *  8H+8L  *  8H+8L  *  8H+8L  *   8H+8L  *  8H+8L *  ##
//head   temp     humi      co        ch4      dust  tail
void    system_send_all_datas( void )
{

    BYTE cmd_write_buffer[40];
    if( ds18b20_a_handle->get_temp_float(ds18b20_a_handle) > temp_max  && \
            gsm_freeze_count > 30
    ) {
        system_send_message();
        gsm_freeze_count = 0;
    }
    ds18b20_a_handle->save_temp_string( ds18b20_a_handle, ds18b20_a_handle->temp_str );

    dht11_handle->get_all_value( dht11_handle );
    adc_handle->get_adc_buffer( adc_handle, CHANNEL_4_MQ2_CH4_SENSOR );
    adc_handle->get_adc_buffer( adc_handle, CHANNEL_5_MQ7_CO_SENSOR );
    adc_handle->get_adc_buffer( adc_handle, CHANNEL_6_DUST_SENSOR );
    cmd_write_buffer[0] = '@';
    cmd_write_buffer[1] = '@';
    memcpy( cmd_write_buffer+2,ds18b20_a_handle->temp_str,7 );
    cmd_write_buffer[8] = '*';
    memcpy( cmd_write_buffer+9,dht11_handle->humi_str, 7 );
    cmd_write_buffer[15] = '*';
    memcpy( cmd_write_buffer+16,adc_handle->channel_4_buffer,7 );
    cmd_write_buffer[22] = '*';
    memcpy( cmd_write_buffer+23,adc_handle->channel_5_buffer,7 );
    cmd_write_buffer[29] = '*';
    memcpy( cmd_write_buffer+30,adc_handle->channel_6_buffer,7 );
    cmd_write_buffer[36] = '*';
    cmd_write_buffer[37] = '#';
    cmd_write_buffer[38] = '#';
    cmd_write_buffer[39] = '\0';
    uart_c_handle->write_string( uart_c_handle, cmd_write_buffer, 40 );
    gsm_freeze_count ++;
}
#define     CHECK   0

ErrorStatus    system_send_message( void )
{
    uart_b_handle->write_string( uart_b_handle, at_sms_config, -1);
    uart_b_handle->write_string( uart_b_handle, at_enter, -1);
    //for( i = 0; i < 10; i ++)
    DELAY_MS(100);
    //uart_a_handle->write_string( uart_a_handle, at_sms_config, -1);
#if CHECK
    while( USART_GetITStatus( uart_b_handle->uart_cfg.usart_num ,USART_IT_RXNE) == RESET );
    if( USART_ReceiveData( uart_b_handle->uart_cfg.usart_num ) != 'O'  ) {
        return ERROR ;
    }
#endif
    uart_b_handle->write_string( uart_b_handle, at_sms_setgsm, -1);
    uart_b_handle->write_string( uart_b_handle, at_enter, -1);
    //for( i = 0; i < 10; i ++)
    DELAY_MS(100);
    //uart_a_handle->write_string( uart_a_handle, at_sms_setgsm, -1);
#if CHECK
    while( USART_GetITStatus( uart_b_handle->uart_cfg.usart_num ,USART_IT_RXNE) == RESET );
    if( USART_ReceiveData( uart_b_handle->uart_cfg.usart_num ) != 'o'  ) {
        return ERROR ;
    }
#endif
    uart_b_handle->write_string( uart_b_handle, at_sms_number, -1);
    uart_b_handle->write_string( uart_b_handle, at_enter, -1);
    //for( i = 0; i < 10; i ++)
    DELAY_MS(100);
    // uart_a_handle->write_string( uart_a_handle, at_sms_number, -1);
#if CHECK
    while( USART_GetITStatus( uart_b_handle->uart_cfg.usart_num ,USART_IT_RXNE) == RESET );

    if( USART_ReceiveData( uart_b_handle->uart_cfg.usart_num ) != 'o'  ) {
        return ERROR ;
    }
#endif
    uart_b_handle->write_string( uart_b_handle, at_sms_content, -1);
    //for( i = 0; i < 10; i ++)
    DELAY_MS(100);
    //uart_a_handle->write_string( uart_a_handle, at_sms_content, -1);
#if CHECK
    while( USART_GetITStatus( uart_b_handle->uart_cfg.usart_num ,USART_IT_RXNE) == RESET );

    if( USART_ReceiveData( uart_b_handle->uart_cfg.usart_num ) != 'o'  ) {
        return ERROR ;
    }
#endif
    uart_b_handle->write_byte( uart_b_handle, at_sms_start);
    uart_b_handle->write_string( uart_b_handle, at_enter, -1);
    //uart_a_handle->write_byte( uart_a_handle, at_sms_start);

    return SUCCESS;
}

u8 Key_Scan(u8 mode)
{
    //key0
    if(Key_Sta[0]==0){
        if(KEY0_HARD_STA){
            delay_ms(10);
            if(KEY0_HARD_STA){
                Key_Sta[0]=1;
                return KEY0_PRES;
            }
        }
    }
    else{
        if(!(KEY0_HARD_STA)){
            Key_Sta[0]=0;
        }
    }
    //KEY1
    if(Key_Sta[1]==0){
        if(KEY1_HARD_STA){
            delay_ms(10);
            if(KEY1_HARD_STA){
                Key_Sta[1]=1;
                return KEY1_PRES;
            }
        }
    }
    else{
        if(!(KEY1_HARD_STA)){
            Key_Sta[1]=0;
        }
    }
    //Key2
    if(Key_Sta[2]==0){
        if(KEY2_HARD_STA){
            delay_ms(10);
            if(KEY2_HARD_STA){
                Key_Sta[2]=1;
                return KEY2_PRES;
            }
        }
    }
    else{
        if(!(KEY2_HARD_STA)){
            Key_Sta[2]=0;
        }
    }

    //KEY3
    if(Key_Sta[3]==0){
        if(KEY3_HARD_STA){
            delay_ms(10);
            if(KEY3_HARD_STA){
                Key_Sta[3]=1;
                return WKUP_PRES;
            }
        }
    }
    else{
        if(!(KEY3_HARD_STA)){
            Key_Sta[3]=0;
        }
    }

    return 0;// 无按键按下
}

//获取按键的当前状态
u8 Key_GetSta(u8 key){
    if(key<=3){
        return Key_Sta[key];
    }
    return 0;
}



void RCC_Configuration(void)
{
    /* Enable system clocks ------------------------------------------------*/
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();                                                                                   // System clock reset.
    RCC_HSEConfig( RCC_HSE_ON );                                                                    // Open the HSE clock.
    HSEStartUpStatus = RCC_WaitForHSEStartUp();                                                     // Wait for HSE clock.
#if 1
    if( HSEStartUpStatus == SUCCESS ) {                                                             // If the HSE time consuming normal.

        FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable );                                     // Enable flash reader buffer.
        FLASH_SetLatency( FLASH_Latency_2 );                                                        // Flash wait state.
        RCC_HCLKConfig( RCC_SYSCLK_Div1 );                                                          // HCLK = SYSCLK = 72.0MHz
        RCC_PCLK2Config( RCC_HCLK_Div1 );                                                           // PCLK2 = HCLK = 72.0MHz
        RCC_PCLK1Config( RCC_HCLK_Div2 );                                                           // PCLK1 = HCLK/2 = 36.0MHz

        RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);                                                                         // Enable PLL

        while( RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );                                       // Wait till PLL is ready.
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );                                                // Select PLL as system clock source.
        while( RCC_GetSYSCLKSource() != 0x08 );                                                     // Wait till PLL is used as system clock source.
    }
    // notice :
    // If there is as "RCC_ADCCLKConfig( RCC_PCLK2_Div6 )" peripheral clock.
    // So, the ADCLK = PCLK2 / 6 = 12MHz.
#endif
    /* Enable peripheral clocks ------------------------------------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD, ENABLE);
    /* Enable DMA1 and DMA2 clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);
    /* Enable ADC1, ADC2, ADC3 and GPIOC clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 |
                           RCC_APB2Periph_ADC3 | RCC_APB2Periph_GPIOC, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1   , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOD ,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void    UART1_INIT( UART* self )
{

    self->init          =   &uart_init;
    self->pin_set       =   &uart_pin_set;
    self->write_byte    =   &uart_write_byte;
    self->write_string  =   &uart_write_string;
    self->clear_buffer  =   &uart_clear_buffer;
    self->fetch_data    =   &uart_fetch_data;

    self->hw.tx_pin                     =   GPIO_Pin_9;
    self->hw.rx_pin                     =   GPIO_Pin_10;
    self->hw.io_clk                     =   RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1;
    self->hw.rx_port                    =   UART_RX_PORT;
    self->hw.tx_port                    =   UART_TX_PORT;
    self->uart_cfg.baud_rate            =   115200;
    self->uart_cfg.word_length          =   USART_WordLength_8b;
    self->uart_cfg.stop_bits            =   USART_StopBits_1;
    self->uart_cfg.parity               =   USART_Parity_No;
    self->uart_cfg.hardware_flow_ctrl   =   USART_HardwareFlowControl_None;
    self->uart_cfg.mode                 =   USART_Mode_Rx | USART_Mode_Tx;
    self->uart_cfg.usart_num            =   USART1;
    self->uart_cfg.uart_id              =   UART_ID_1;
    self->init( self );

}

void    UART2_INIT( UART* self )
{
    self->init          =   &uart_init;
    self->pin_set       =   &uart_pin_set;
    self->write_byte    =   &uart_write_byte;
    self->write_string  =   &uart_write_string;
    self->clear_buffer  =   &uart_clear_buffer;
    self->fetch_data    =   &uart_fetch_data;

    self->hw.tx_pin                     =   GPIO_Pin_2;
    self->hw.rx_pin                     =   GPIO_Pin_3;
    self->hw.io_clk                     =   RCC_APB1Periph_USART2;
    self->hw.rx_port                    =   UART_RX_PORT;
    self->hw.tx_port                    =   UART_TX_PORT;
    self->uart_cfg.baud_rate            =   115200;
    self->uart_cfg.word_length          =   USART_WordLength_8b;
    self->uart_cfg.stop_bits            =   USART_StopBits_1;
    self->uart_cfg.parity               =   USART_Parity_No;
    self->uart_cfg.hardware_flow_ctrl   =   USART_HardwareFlowControl_None;
    self->uart_cfg.mode                 =   USART_Mode_Rx | USART_Mode_Tx;
    self->uart_cfg.usart_num            =   USART2;
    self->uart_cfg.uart_id              =   UART_ID_2;

    self->init( self );

}

void    UART3_INIT( UART* self )
{
    self->init          =   &uart_init;
    self->pin_set       =   &uart_pin_set;
    self->write_byte    =   &uart_write_byte;
    self->write_string  =   &uart_write_string;
    self->clear_buffer  =   &uart_clear_buffer;
    self->fetch_data    =   &uart_fetch_data;

    self->hw.tx_pin                     =   GPIO_Pin_10;
    self->hw.rx_pin                     =   GPIO_Pin_11;
    self->hw.io_clk                     =   RCC_APB1Periph_USART3;
    self->hw.rx_port                    =   GPIOB;
    self->hw.tx_port                    =   GPIOB;
    self->uart_cfg.baud_rate            =   9600;
    self->uart_cfg.word_length          =   USART_WordLength_8b;
    self->uart_cfg.stop_bits            =   USART_StopBits_1;
    self->uart_cfg.parity               =   USART_Parity_No;
    self->uart_cfg.hardware_flow_ctrl   =   USART_HardwareFlowControl_None;
    self->uart_cfg.mode                 =   USART_Mode_Rx | USART_Mode_Tx;
    self->uart_cfg.usart_num            =   USART3;
    self->uart_cfg.uart_id              =   UART_ID_3;

    self->init( self );
}

void    ADC_INIT( ADC *this )
{

    this->cfg.adc_mode               =   ADC_Mode_Independent;
    this->cfg.adc_num                =   ADC1;
    this->cfg.continuous_conv_mode   =    DISABLE;
    this->cfg.external_trig_conv     =   ADC_ExternalTrigConv_None;
    this->cfg.data_align             =   ADC_DataAlign_Right;
    this->cfg.scan_conv_mode         =   DISABLE;

    this->hw.adc_module_clk          =      RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1;
    this->hw.adc_port                =      GPIOA;
    this->hw.channel_4_pin           =      GPIO_Pin_4;
    this->hw.channel_5_pin           =      GPIO_Pin_5;
    this->hw.channel_6_pin           =      GPIO_Pin_6;

    this->init  =   &adc_init;
    this->reset =   &adc_reset;
    this->pin_set   =   &adc_pin_set;
    this->float_2_ascii =   &adc_float_2_ascii;
    this->get_adc_value =   &adc_get_adc_value;
    this->get_adc_buffer =   &adc_get_adc_buffer;

    this->init( this );
}

void    DS18B20_INIT( DS18B20 *dev_p )
{

    dev_p->init             =   &ds18b20_init;
    dev_p->float_2_ascii    =   &ds18b20_float_2_ascii;
    dev_p->get_temp_float   =   &ds18b20_get_temp;
    dev_p->hw.set_high      =   &ds18b20_hw_set_high;
    dev_p->hw.set_low       =   &ds18b20_hw_set_low;
    dev_p->hw.read_line     =   &ds18b20_hw_read_line;
    dev_p->read_byte        =   &ds18b20_read_byte;
    dev_p->reset            =   &ds18b20_reset;
    dev_p->save_temp_string =   &ds18b20_save_temp_string;
    dev_p->write_byte       =   &ds18b20_write_byte;
    dev_p->pin_set          =   &ds18b20_pin_set;
    dev_p->presence         =   &ds18b20_presence;
    dev_p->start            =   &ds18b20_start;

    dev_p->hw.line_port     =   GPIOD;
    dev_p->hw.line_pin      =   GPIO_Pin_7;
    dev_p->hw.line_io_clk   =   RCC_APB2Periph_GPIOD;

    dev_p->init(dev_p);

}

void    DHT11_INIT( DHT11 *p_dev )
{
    p_dev->init             =   &dht11_init;
    p_dev->reset            =   &dht11_reset;
    p_dev->pin_set          =   &dht11_pin_set;
    p_dev->scan             =   &dht11_scan;
    p_dev->read_line        =   &dht11_read_line;
    p_dev->read_bit         =   &dht11_read_bit;
    p_dev->read_byte        =   &dht11_read_byte;
    p_dev->read_sensor      =   &dht11_read_sensor;
    p_dev->get_temp_float   =   &dht11_get_temp_float;
    p_dev->get_humi_float   =   &dht11_get_humi_float;
    p_dev->get_all_value    =   &dht11_get_all_value;
    p_dev->float_2_ascii    =   &dht11_float_2_ascii;

    p_dev->hw.line_pin      =   GPIO_Pin_6;
    p_dev->hw.line_port     =   GPIOD;
    p_dev->hw.line_io_clk   =   RCC_APB2Periph_GPIOD;

    p_dev->init(p_dev);
}

void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(LED0_GPIO_RCC_CLK|LED1_GPIO_RCC_CLK, ENABLE);    //使能PB,PE端口时钟

    GPIO_InitStructure.GPIO_Pin = LED0_PIN;                       //LED0端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //IO口速度为50MHz
    GPIO_Init(LED0_GPIO, &GPIO_InitStructure);                    //根据设定参数初始化LED0
    GPIO_SetBits(LED0_GPIO,LED0_PIN);                               //LED0输出高

    GPIO_InitStructure.GPIO_Pin = LED1_PIN;                     //LED1端口配置, 推挽输出
    GPIO_Init(LED1_GPIO, &GPIO_InitStructure);                  //推挽输出 ，IO口速度为50MHz
    GPIO_SetBits(LED1_GPIO,LED1_PIN);                               //LED1输出高
}




//按键初始化函数
void Key_Init(void) //IO初始化
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(KEY0_RCC_CLK|KEY1_RCC_CLK|KEY2_RCC_CLK|KEY3_RCC_CLK,ENABLE);//使能PORTA,PORTE时钟

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;//KEY0-KEY2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
    GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE2,3,4

    //初始化 WK_UP-->GPIOA.0     下拉输入
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0设置成输入，默认下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.0

}
