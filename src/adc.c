/*
 * adc.c
 *
 *  Created on: 2018Äê4ÔÂ16ÈÕ
 *      Author: weihaochen
 */
/*
 *
 *      PA4 - ADC1 - Channel4       - MQ2
 *      PA5 - ADC1 - Channel5       - MQ7
 *      PA6 - ADC1 - Channel6       - ·Û³¾
 *
 */
#include "global.h"
#include "stm32f10x_adc.h"




void    adc_init( ADC *this )
{

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    this->cfg.adc_cfg.ADC_ContinuousConvMode  =   this->cfg.continuous_conv_mode;
    this->cfg.adc_cfg.ADC_DataAlign            =   this->cfg.data_align;
    this->cfg.adc_cfg.ADC_ExternalTrigConv      =   this->cfg.external_trig_conv;
    this->cfg.adc_cfg.ADC_Mode                  =   this->cfg.adc_mode;
    this->cfg.adc_cfg.ADC_NbrOfChannel          =   this->cfg.nbr_of_channel;
    this->cfg.adc_cfg.ADC_ScanConvMode          =   this->cfg.scan_conv_mode;
    ADC_Init( this->cfg.adc_num, &this->cfg.adc_cfg );
    ADC_Cmd(this->cfg.adc_num, ENABLE);
    ADC_ResetCalibration(this->cfg.adc_num);
    while(ADC_GetResetCalibrationStatus(this->cfg.adc_num));
    ADC_StartCalibration(this->cfg.adc_num);
    while(ADC_GetCalibrationStatus(this->cfg.adc_num));
}

void    adc_reset( ADC *this )
{

}

void    adc_pin_set( ADC *this )
{

    RCC_APB2PeriphClockCmd( this->hw.adc_module_clk, ENABLE );

    this->hw.gpio_cfg.GPIO_Pin = this->hw.channel_4_pin | this->hw.channel_5_pin | this->hw.channel_6_pin;
    this->hw.gpio_cfg.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init( this->hw.adc_port , &this->hw.gpio_cfg);
}

float    adc_get_adc_value( ADC *this, ADC_CHANNEL channel )
{
    uint16  adc_data = 0;
    float   adc_value = 0.0f;
    ADC_RegularChannelConfig(this->cfg.adc_num, channel, 1, ADC_SampleTime_239Cycles5 );
    ADC_SoftwareStartConvCmd(this->cfg.adc_num, ENABLE);
    while(!ADC_GetFlagStatus(this->cfg.adc_num, ADC_FLAG_EOC ));
    adc_data    =   ADC_GetConversionValue(this->cfg.adc_num);
    adc_value   =   (( (float)adc_data / 4096.0f   )) * 3.3f;
    if  ( channel == CHANNEL_4_MQ2_CH4_SENSOR) {
        this->channel_4_value   =   adc_value;
    }else if    ( channel == CHANNEL_5_MQ7_CO_SENSOR ) {
        this->channel_5_value   =   adc_value;
    }else if    ( channel == CHANNEL_6_DUST_SENSOR ) {
        this->channel_6_value   =   adc_value;
    }
    return adc_value;
}

void    adc_get_adc_buffer( ADC *this, ADC_CHANNEL channel )
{
    uint16  adc_data = 0;
    float   adc_value = 0.0f;
    ADC_RegularChannelConfig(this->cfg.adc_num, channel, 1, ADC_SampleTime_239Cycles5 );
    ADC_SoftwareStartConvCmd(this->cfg.adc_num, ENABLE);
    while(!ADC_GetFlagStatus(this->cfg.adc_num, ADC_FLAG_EOC ));
    adc_data    =   ADC_GetConversionValue(this->cfg.adc_num);
    if  ( channel == CHANNEL_4_MQ2_CH4_SENSOR) {
        adc_value   =   (( (float)adc_data / 4096.0f   )) * (10000.0/3.3f);
        this->channel_4_value   =   adc_value;
        this->float_2_ascii( this, adc_value, this->channel_4_buffer );
    }else if    ( channel == CHANNEL_5_MQ7_CO_SENSOR ) {
        adc_value   =   (( (float)adc_data / 4096.0f   )) * (1000.0f/3.3f);
        this->channel_5_value   =   adc_value;
        this->float_2_ascii( this, adc_value, this->channel_5_buffer );
    }else if    ( channel == CHANNEL_6_DUST_SENSOR ) {
        adc_value   =   (( (float)adc_data / 1024.0f   )) * 4200.0;
        this->channel_6_value   =   adc_value;
        this->float_2_ascii( this, adc_value, this->channel_6_buffer );
    }


}

void    adc_float_2_ascii( ADC *self, float f_data, uint8 *fltBuffer)
{

    if (f_data < 0) {
        fltBuffer[0] = (long)'-';
        f_data = -f_data;
    } else {
        fltBuffer[0] = (long)' ';
    }
    if (f_data < 999 && f_data > 99) {
        fltBuffer[1] = (long)((long)(f_data) / 100) % 1000 + 0x30;
        fltBuffer[2] = (long)((long)(f_data) / 10) % 10 + 0x30;
        fltBuffer[3] = (long)((long)(f_data) / 1) % 10 + 0x30;
        fltBuffer[4] = (long)('.');
        fltBuffer[5] = (long)(f_data * 10) % 10 + 0x30;
        fltBuffer[6] = (long)(f_data * 100) % 10 + 0x30;
        fltBuffer[7] = (long)'\0';
    } else if (f_data < 99 && f_data > 9) {
        fltBuffer[1] = (long)' ';
        fltBuffer[2] = (long)((long)(f_data) / 10) % 10 + 0x30;
        fltBuffer[3] = (long)((long)(f_data) / 1) % 10 + 0x30;
        fltBuffer[4] = (long)('.');
        fltBuffer[5] = (long)(f_data * 10) % 10 + 0x30;
        fltBuffer[6] = (long)(f_data * 100) % 10 + 0x30;
        fltBuffer[7] = (long)'\0';
    } else if (f_data < 9) {
        fltBuffer[1] = (long)' ';
        fltBuffer[2] = (long)' ';
        fltBuffer[3] = (long)((long)(f_data) / 1) % 10 + 0x30;
        fltBuffer[4] = (long)('.');
        fltBuffer[5] = (long)(f_data * 10) % 10 + 0x30;
        fltBuffer[6] = (long)(f_data * 100) % 10 + 0x30;
        fltBuffer[7] = (long)'\0';
    }
}

