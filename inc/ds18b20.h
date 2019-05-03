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

#ifndef DS18B20_H_
#define DS18B20_H_

#include "stm32f10x_gpio.h"
#include "type.h"
#include "sys.h"


#define             LINE_HIGH(self)                     ( GPIO_SetBits(self->line_port,self->line_pin) )
#define             LINE_LOW(self)                      ( GPIO_ResetBits(self->line_port,self->line_pin) )
#define             LINE_DATA(self)                     ( GPIO_ReadInputDataBit(self->line_port, self->line_pin) )


struct  ds18b20_hw_t{

    uint16              line_pin;
    GPIO_TypeDef        *line_port;
    GPIO_InitTypeDef    line_gpio_cfg;
    struct              ds18b20_hw_t *self;
    uint32              line_io_clk;

    void      (*set_high)   ( struct ds18b20_hw_t *self );
    void      (*set_low)    ( struct ds18b20_hw_t *self );
    uint8     (*read_line)  ( struct ds18b20_hw_t *self );
};

typedef struct  ds18b20_t{

    struct ds18b20_hw_t hw;
    struct ds18b20_t    *self;
    float   temp_value;
    uint8   temp_str[8];
    uint8   read_buffer[16];
    uint8   *op;
    uint8   id_buffer[8];

    ErrorStatus     (*init)             ( struct ds18b20_t *self );
    void            (*reset)            ( struct ds18b20_t *self );
    void            (*write_byte)       ( struct ds18b20_t *self, uint8 val );
    uint8           (*read_byte)        ( struct ds18b20_t *self );
    float           (*get_temp_float)   ( struct ds18b20_t *self );
    void            (*save_temp_string) ( struct ds18b20_t *self, uint8 *dis_str );
    void            (*float_2_ascii)    ( struct ds18b20_t *self, float val, uint8 *ascii_str );
    void            (*pin_set)          ( struct ds18b20_t *self );
    ErrorStatus     (*presence)         ( struct ds18b20_t *self );
    ErrorStatus     (*start)            ( struct ds18b20_t *self );

} DS18B20;


extern  ErrorStatus     ds18b20_init            ( struct ds18b20_t *self );
extern  void            ds18b20_float_2_ascii   ( struct ds18b20_t *self, float f_data, uint8 *fltBuffer);
extern  void            ds18b20_save_temp_string( struct ds18b20_t *self , uint8 *dis_str);
extern  float           ds18b20_get_temp        ( struct ds18b20_t *self );
extern  uint8           ds18b20_read_byte       ( struct ds18b20_t *self );
extern  void            ds18b20_write_byte      ( struct ds18b20_t *self, uint8 val );
extern  void            ds18b20_reset           ( struct ds18b20_t   *self );
extern  uint8           ds18b20_hw_read_line    ( struct ds18b20_hw_t *self );
extern  void            ds18b20_hw_set_low      ( struct ds18b20_hw_t *self );
extern  void            ds18b20_hw_set_high     ( struct ds18b20_hw_t *self );
extern  void            ds18b20_pin_set         ( struct ds18b20_t *self );
extern  ErrorStatus     ds18b20_presence        ( struct ds18b20_t *self );
extern  ErrorStatus     ds18b20_start           ( struct ds18b20_t *self );

#endif /* DS18B20_H_ */
