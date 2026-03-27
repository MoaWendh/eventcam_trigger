#include <string>
#include <gpiod.h>
#include <atomic>
#include <thread>
#include <iostream>


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
        const int lineA = 43;   // PH.00  - Pino 33 (Pisca Led)  
        const int lineB = 85;   // PN.01  - Pino 15 (Controle Motor_01)
        const int lineC = 105;  // PQ.05  - Pino 29 (Controle Motor_02)
        const int lineD = 41;   // PG.06  - Pino 32 (Controle Motor_03)
        const int lineE = 122;  // PY.00  - Pino 13 (Controle Motor_04)
        const int lineF = 144;  // PAC.06 - Pino 7  (Trigger camera de eventos)
        const int lineG = 106;  // PQ.06  - Pino 31 (Trigger camera convencional)
        const int lineH = 126;  // PY.04  - Pino 16 (Controle laser)
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




// ESta classe LedContreoller serve apenas para piscar o led pelo barremano de IO da Jetso:
class LEDController {
private:
    std::atomic<bool> is_running{false};
    std::thread blink_thread;
    struct gpiod_line *line; // Guarda o pino do barramento da Jetson 
    int pulse_duration_ms; // Armazena a duração internamente

    void run_blink();

public:
    // Construtor configurando pino e tempo:
    LEDController(struct gpiod_line *gpio_line, int duration_ms) 
        : line(gpio_line), pulse_duration_ms(duration_ms) {}

    // Desturtor do objeto:
    ~LEDController() {
        stop();
    }

    void start() {
        if (is_running) {
            std::cout << "Pulso Led ativado.\n";
            return;
        }
        is_running = true;
        blink_thread = std::thread(&LEDController::run_blink, this);
    }

    void stop() {
        if (is_running) {
            is_running = false;
            if (blink_thread.joinable()) {
                blink_thread.join();
            }
        } else {
            std::cout << "Nenhuma thread ativa para parar!!!!\n";
        }
    }

    bool isActive() const {
        return is_running;
    }
};
