
// Project.
/****************************************************************************/
/*                                                                          */
/*  @file       : DS18B20.c                                                 */
/*  @Copyright  : Lifi.MLT (c) 2015 MULTIBEANS_ORG All rights reserved. 	*/
/*  @Revision   : Ver 1.0.                                                  */
/*  @Data       : 2018.01.15 Realse.                                        */
/*  @Belong     : PROJECT.                                                  */
/*                                                                          */
/*  **code : (GBK/GB2312) in Windows 10_x64. CCS v6.1 platform.             */
/****************************************************************************/
/*  @Attention:                                                             */
/*  ---------------------------------------------------------------------   */
/*  |    Data    |  Behavior |     Offer     |          Content         |   */
/*  | 2018.01.15 |   create  |DelvisBeans(M) | ceate the document.      |   */
/*  ---------------------------------------------------------------------   */
/*                                                            MULTIBEANS.   */
/****************************************************************************/

/**
 * \brief   This project about ds18b20 on common channels DS18B20 .
 *
 * \License  THIS FILE IS PART OF MULTIBEANS PROJECT ;
 *           all of the files  - The core part of the project;
 *           THIS PROGRAM IS FREE SOFTWARE, JUST NEED GPL 3.0 LICENSE;
 *           YOU SHOULD HAVE RECEIVED A COPY OF WTFPL LICENSE, IF NOT,
 *           MULTIBEANS WILL TAKE APPROPRIATE MEASURES.
 *
 *           * You can download the license on our Github. ->
 *           * -> https://github.com/multibeans  <-
 *           * Copyright (c) 2013-2018 MULTIBEANS ORG. http://www.mltbns.org/
 *           * Copyright (c) 2018 Wei Haochen.  Email: carlos@mltbns.top
 *
 *  \note    void.
 */

/**
 * \brief    connection.
 *
 * \offer    Wei Haochen
 *
 *
 * \return   None.
 *
 *              MSP430
 *             ---------------
 *            |               |
 *   line <-->|P1.0        	  |
 *			  |               |
 *
 */


/*
 * ds18b20.c
 *
 *
 * you should setup this function in main.
struct  ds18b20_t  dev, *p_dev;
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

    dev_p->hw.line_port     =   GPIOA;
    dev_p->hw.line_pin      =   GPIO_Pin_0;
    dev_p->hw.line_io_clk   =   RCC_APB2Periph_GPIOA;

    if ( (dev_p->init(dev_p) ) == ERROR ) {
        uart_a_handle->write_string(uart_a_handle, "temp sensor init error! \n\r",-1);
    }else {
        uart_a_handle->write_string(uart_a_handle, "temp sensor init ok! \n\r",-1);
    }

}

 */

#include "global.h"




static void    SET_LINE_OUT( struct ds18b20_hw_t *self )
{

    self->line_gpio_cfg.GPIO_Mode    =   GPIO_Mode_Out_OD;
    GPIO_Init(self->line_port, &self->line_gpio_cfg );

}
static void    SET_LINE_IN( struct ds18b20_hw_t *self )
{

    self->line_gpio_cfg.GPIO_Mode    =   GPIO_Mode_IN_FLOATING;
    GPIO_Init(self->line_port, &self->line_gpio_cfg );

}

void    ds18b20_hw_set_high( struct ds18b20_hw_t *self )
{
    LINE_HIGH(self);
}

void    ds18b20_hw_set_low( struct ds18b20_hw_t *self )
{
    LINE_LOW(self);
}

ErrorStatus    ds18b20_init( struct ds18b20_t *self )
{

    self->pin_set( self );
    self->reset( self );
    if( self->presence( self ) == ERROR ) {
        return ERROR;
    }else {
        return SUCCESS;
    }

}
ErrorStatus ds18b20_presence( struct ds18b20_t *self )
{
    uint8_t pulse_time = 0;

    SET_LINE_IN((&self->hw));
    while( LINE_DATA((&self->hw)) && pulse_time < 200 )
    {
        pulse_time++;
        DELAY_US(1);
    }
    /* 经过100us后，存在脉冲都还没有到来*/
    if( pulse_time >= 200 )
        return ERROR;
    else
        pulse_time = 0;

    /* 存在脉冲到来，且存在的时间不能超过240us */
    while( !LINE_DATA((&self->hw)) && pulse_time < 240 )
    {
        pulse_time++;
        DELAY_US(1);
    }
    if( pulse_time >=240 )
        return ERROR;
    else
        return SUCCESS;
}

void    ds18b20_pin_set( struct ds18b20_t *self )
{

    RCC_APB2PeriphClockCmd(self->hw.line_io_clk,ENABLE);
    self->hw.line_gpio_cfg.GPIO_Pin     =   self->hw.line_pin;
    self->hw.line_gpio_cfg.GPIO_Mode    =   GPIO_Mode_Out_PP;
    self->hw.line_gpio_cfg.GPIO_Speed   =   GPIO_Speed_50MHz;
    GPIO_Init(self->hw.line_port, &self->hw.line_gpio_cfg );
}

void    ds18b20_reset( struct  ds18b20_t   *self )
{
    SET_LINE_OUT(&self->hw);
    self->hw.set_low( &self->hw );
    DELAY_US(750);
    self->hw.set_high( &self->hw );
    DELAY_US(15);
}

uint8   ds18b20_hw_read_line( struct ds18b20_hw_t *self )
{

    SET_LINE_OUT(self);
    LINE_LOW(self);
    DELAY_US(2);
    LINE_HIGH(self);

    SET_LINE_IN(self);
    DELAY_US(12);
    if( LINE_DATA(self) == 1 ) {
        DELAY_US(50);
        return 1;
    }else{
        DELAY_US(50);
        return 0;
    }
}

void    ds18b20_write_byte( struct ds18b20_t *self, uint8 val )
{
    uint8   i;
    uint8   tbit;
    SET_LINE_OUT(&self->hw);

    for ( i = 1; i <= 8; i++ ) {
        tbit    =   val&0x01;
        val =   val >> 1;
        if ( tbit == 1 ) {
            self->hw.set_low( &self->hw );
            DELAY_US(2);
            self->hw.set_high( &self->hw );
            DELAY_US(60);
        }else {
            self->hw.set_low( &self->hw );
            DELAY_US(60);
            self->hw.set_high( &self->hw );
            DELAY_US(2);
        }
    }
}


uint8   ds18b20_read_byte( struct ds18b20_t *self )
{
    uint8  i,u = 0;
    uint8  dat = 0;

    for( i = 0 ; i < 8 ; i ++) {
        u   =   self->hw.read_line( &self->hw );
        dat =   (u<<7)|(dat>>1);
    }
    return(dat);
}

ErrorStatus ds18b20_start( struct ds18b20_t *self )
{
    self->reset( self );
    if( self->presence( self ) == ERROR ) {
        return ERROR;
    }
    self->write_byte( self, 0xcc );
    self->write_byte( self, 0x44 );
    return SUCCESS;
}


float   ds18b20_get_temp( struct ds18b20_t *self )
{
    float   float_temp = 0;
    short   short_temp = 0;
    uint8   uint8_temp = 0;
    uint8   TL = 0,TH = 0;

    self->start( self );
    self->reset( self );

    if( self->presence( self ) == ERROR ) {
        return 0;
    }

    self->write_byte( self, 0xcc );
    self->write_byte( self, 0xbe );
    DELAY_US(100);
    TL  =   self->read_byte( self );
    TH  =   self->read_byte( self );

    if ( TH > 7 ) {
        TH  =   ~TH;
        TL  =   ~TL;
        uint8_temp  =   0;
    }else {
        uint8_temp  =   1;
    }
    short_temp     =    TH;
    short_temp     <<= 8;
    short_temp     +=   TL;
    short_temp      =   (float)short_temp * 0.625;
    float_temp      =   (float)short_temp / 10;

    return float_temp;

}

void    ds18b20_save_temp_string( struct ds18b20_t *self, uint8 *dis_str )
{

    float   temp;
    temp = self->get_temp_float( self );
    self->float_2_ascii( self, temp, self->temp_str );
    memcpy( dis_str,self->temp_str ,8);


}

void    ds18b20_float_2_ascii( struct ds18b20_t *self, float f_data, uint8 *fltBuffer)
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

