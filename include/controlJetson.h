#include <string>
#include <gpiod.h>


// Struct para as linas, pinos, da Jetson:
struct GPIO_Lines {
    struct gpiod_line *triggerEventCam;
    struct gpiod_line *triggerNormalCam;
    struct gpiod_line *piscaLed;
    struct gpiod_line *controlLaser;
    struct gpiod_line *controlMotor01;
    struct gpiod_line *controlMotor02;
    struct gpiod_line *controlMotor03;
    struct gpiod_line *controlMotor04;
};


// Classe com métodos para configurar a Jetson Orin nano:
class configJetson {
private:
    // Identificador do controlador GPIO no JetPack 6
    const std::string chipIO = "gpiochip0";

    // Inicializa as linhas do GPIO da Jetson relativas aso pinos do barramento conector J12:
    struct LineJetson {
        const int lineA = 43;   // PH.00  - Pino 33
        const int lineB = 85;   // PN.01  - Pino 15
        const int lineC = 105;  // PQ.05  - Pino 29
        const int lineD = 41;   // PG.06  - Pino 32
        const int lineE = 122;  // PY.00  - Pino 13
        const int lineF = 144;  // PAC.06 - Pino 7
        const int lineG = 106;  // PQ.06  - Pino 31
        const int lineH = 126;  // PY.04  - Pino 16
    };

    // Pinos fisicos do barramento  GPIO J12 da Jetson Orin nano:
    struct PinoJetson {
        const int header_pinA = 33; // Pisca Led  
        const int header_pinB = 15; // Controle Motor_01 
        const int header_pinC = 29; // Controle Motor_02   
        const int header_pinD = 32; // Controle Motor_03
        const int header_pinE = 13; // Controle Motor_04      
        const int header_pinF = 7;  // Trigger camera de eventos   
        const int header_pinG = 31; // Trigger camera convencional
        const int header_pinH = 16; // Controle laser
    };

    LineJetson lines;
    PinoJetson pinos;

public: 
    // Este método é chamado para configurar o barramento GPIO da Jetson, apenas isso:
    GPIO_Lines configura_GPIO_Jetson(struct gpiod_chip **chip_ptr); 

    void liberaGPIO_Jetson(struct gpiod_chip *chip, GPIO_Lines gpios);

    // Métodos Get para capturar as informações dos pinos configurados e lines:
    LineJetson getLines() { 
        return lines; 
    }

    PinoJetson getPinos() { 
        return pinos; 
    }

};