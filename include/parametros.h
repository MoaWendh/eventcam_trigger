#ifndef CONFIG_PARAMETROS_H
#define CONFIG_PARAMETROS_H

#include <cstdint>

struct PARAMETROS_GERAIS {
    // Parametros que definem as caracterísicas do triiger por HW da camera: 
    int64_t duracao_pulso_trigger   = 1000000; // em micro segundos
    int64_t duracao_pre_trigger     = 50000;   // em micro segundos
    int64_t duracao_pos_trigger     = 50000;   // em micro segundos
    int64_t duracao_led             = 200000;  // em micro segundos
    int numero_ciclos_trigger       = 1;

    // Núemros de séries das cameras convencionais
    const std::string serialNumber_conv_cam_01= "25083333";
    const std::string serialNumber_conv_cam_02= "00000414";  
    
    // Definição dos paths referentes aos chips de IO da Jeson que geram o PWM: 
    const std::string channelToExport_A= "/sys/class/pwm/pwmchip3/";  
    const std::string channelToExport_B= "/sys/class/pwm/pwmchip2/"; 

    // Definição do Duty-cicle dos PWMs, valor em percentual.
    long dutyCicle_PWM_A= 5;   //  PWM referente ao controle do duty cicle para blink led.
    long dutyCicle_PWM_B= 10;  // PWM referente ao controle da tensão.

    // Definição do periodo dos PWMs, valor em nano segundos.
    long periodo_PWM_A= 100000000;  //  A: referente ao blink do led (pino 32 da Jetson). 
    long periodo_PWM_B= 1000000;    // B: referente a tensão do led (pino 33 da Jetson).     
    
    // Números de série das câmeras de eventos:
    const std::string serialNumber_event_cam0= "00000414"; // HD
    const std::string serialNumber_event_cam2= "00000679"; // VGA
    const std::string serialNumber_event_cam3= "00000680"; // VGA 
    
    // Define se está usando o led de potencia LT2PR da Opto Engineering
    bool useLed_LT2PR= true;
};

#endif