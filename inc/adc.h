/*
 *
 *      PA4 - ADC1 - Channel4       - MQ2
 *      PA5 - ADC1 - Channel5       - MQ7
 *      PA6 - ADC1 - Channel6       - ·Û³¾
 *
 */

#ifndef ADC_H_
#define ADC_H_


typedef     enum    ADC_CH {

    CHANNEL_4_MQ2_CH4_SENSOR = 4,
    CHANNEL_5_MQ7_CO_SENSOR = 5,
    CHANNEL_6_DUST_SENSOR = 6

} ADC_CHANNEL ;



struct  adc_hw_t {

    uint16              channel_4_pin;
    uint16              channel_5_pin;
    uint16              channel_6_pin;
    uint32              adc_module_clk;
    GPIO_TypeDef        *adc_port;
    struct              dht11_hw_t *self;
    GPIO_InitTypeDef    gpio_cfg;
};

struct  adc_cfg_t {

    ADC_InitTypeDef     adc_cfg;
    uint32              adc_mode;
    FunctionalState     scan_conv_mode;
    FunctionalState     continuous_conv_mode;
    uint32              external_trig_conv;
    uint32              data_align;
    uint8               nbr_of_channel;
    ADC_TypeDef         *adc_num;
};

typedef struct  adc_t {

    ADC_CHANNEL         adc_channel;
    struct  adc_hw_t    hw;
    struct  adc_t       *self;
    struct  adc_cfg_t   cfg;

    BYTE    channel_4_buffer[10];
    BYTE    channel_5_buffer[10];
    BYTE    channel_6_buffer[10];
    float   channel_4_value;
    float   channel_5_value;
    float   channel_6_value;


    void    (*init)( struct adc_t   *self );
    void    (*reset)( struct adc_t  *self );
    void    (*pin_set)( struct adc_t *self );
    float   (*get_adc_value)( struct    adc_t *self, ADC_CHANNEL channel );
    void    (*get_adc_buffer)( struct   adc_t *self, ADC_CHANNEL channel );
    void    (*float_2_ascii)(struct   adc_t *self, float f_data, uint8 *fltBuffer);


} ADC;


extern void    adc_init( ADC *this );
extern void    adc_reset( ADC *this );
extern void    adc_pin_set( ADC *this );
extern float    adc_get_adc_value( ADC *this, ADC_CHANNEL channel );
extern void    adc_get_adc_buffer( ADC *this, ADC_CHANNEL channel);
extern void    adc_float_2_ascii( ADC *self, float f_data, uint8 *fltBuffer);

#endif /* ADC_H_ */
