#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H
//BOARD_CUSTOM
//BOARD_EXG_V3
#ifdef BOARD_2CH_ECG_RAT
//External LFCLK
//#define NRF_CLOCK_LFCLKSRC           \
//  { .source = NRF_CLOCK_LF_SRC_XTAL, \
//    .rc_ctiv = 0,                    \
//    .rc_temp_ctiv = 0,               \
//    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM }
//Internal LFCLK
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_RC,            \
                                 .rc_ctiv       = 16,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = 0}

//#define LED_1 9
#define LED_2 10
#define BATTERY_AIN_PIN 5
#define BATTERY_LOAD_SWITCH_CTRL_PIN 4
//see SDK Config for BLE_BAS_ENABLED
//1. SAADC_ENABLED


#endif
#endif