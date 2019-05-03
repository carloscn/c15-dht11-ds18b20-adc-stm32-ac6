/*
 * dht11.h
 *
 *  Created on: 2018Äê4ÔÂ15ÈÕ
 *      Author: weihaochen
 */

#ifndef DHT11_H_
#define DHT11_H_

#include "stm32f10x_gpio.h"
#include "type.h"
#include "sys.h"


#define       DHT11_LINE_IN(this)                     this->hw.line_gpio_cfg.GPIO_Mode    =   GPIO_Mode_IN_FLOATING;      \
                                                      GPIO_Init(this->hw.line_port, &this->hw.line_gpio_cfg )
#define       DHT11_LINE_OUT(this)                    this->hw.line_gpio_cfg.GPIO_Mode    =   GPIO_Mode_Out_OD;  \
                                                      GPIO_Init(this->hw.line_port, &this->hw.line_gpio_cfg )
#define       DHT11_LINE_HIGH(this)                   ( GPIO_SetBits(this->hw.line_port,this->hw.line_pin) )
#define       DHT11_LINE_LOW(this)                    ( GPIO_ResetBits(this->hw.line_port,this->hw.line_pin) )
#define       DHT11_LINE_READ(this)                   ( GPIO_ReadInputDataBit(this->hw.line_port, this->hw.line_pin) )

struct  dht11_hw_t {

    uint16              line_pin;
    GPIO_TypeDef        *line_port;
    GPIO_InitTypeDef    line_gpio_cfg;
    struct              dht11_hw_t *self;
    uint32              line_io_clk;

    void      (*set_high)   ( struct dht11_hw_t *self );
    void      (*set_low)    ( struct dht11_hw_t *self );
    uint8     (*read_line)  ( struct dht11_hw_t *self );

};

typedef struct  dht11_t{

    struct dht11_hw_t hw;
    struct dht11_t    *self;

    float   temperature_value;
    float   humidity_value;
    uint8   temp_str[8];
    uint8   humi_str[8];
    uint8   save_buffer[16];
    uint8   *op;
    uint8   id_buffer[8];

    ErrorStatus     (*init)             ( struct dht11_t *self );
    void            (*reset)            ( struct dht11_t *self );
    void            (*write_byte)       ( struct dht11_t *self, uint8 val );
    uint8           (*read_byte)        ( struct dht11_t *self );
    float           (*get_temp_float)   ( struct dht11_t *self );
    float           (*get_humi_float)   ( struct dht11_t *self );
    void            (*float_2_ascii)    ( struct dht11_t *self, float val, uint8 *ascii_str );
    void            (*pin_set)          ( struct dht11_t *self );
    ErrorStatus     (*scan)             ( struct dht11_t *self );
    uint8           (*read_line)        ( struct dht11_t *self );
    ErrorStatus     (*read_sensor)      ( struct dht11_t *self );
    ErrorStatus     (*get_all_value)    ( struct dht11_t *self );
    FlagStatus      (*read_bit)         ( struct dht11_t *self );
} DHT11;


extern ErrorStatus      dht11_init( DHT11 *this );
extern void             dht11_reset( DHT11 *this );
extern void             dht11_pin_set( DHT11 *this );
extern ErrorStatus      dht11_scan( DHT11 *this );
extern uint8            dht11_read_line( DHT11 *this );
extern FlagStatus       dht11_read_bit( DHT11 *this );
extern uint16           dht11_read_byte( DHT11 *this );
extern ErrorStatus      dht11_read_sensor( DHT11 *this );
extern float            dht11_get_temp_float( DHT11 *this );
extern float            dht11_get_humi_float( DHT11 *this );
extern ErrorStatus      dht11_get_all_value( DHT11 *this );
extern void             dht11_float_2_ascii( struct ds18b20_t *self, float f_data, uint8 *fltBuffer);

#endif /* DHT11_H_ */
