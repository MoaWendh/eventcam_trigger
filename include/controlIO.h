#include <string>
#include <gpiod.h>
#include <atomic>
#include <thread>
#include <iostream>
#include <fstream>


// Struct para as linhas, pinos, da Jetson:
struct GPIO_Lines {
    struct gpiod_line *triggerEventCam;
    struct gpiod_line *triggerNormalCam;
    struct gpiod_line *piscaLed;
    struct gpiod_line *controlLaser;
    struct gpiod_line *controlMotor01;
    struct gpiod_line *controlMotor02;
};


// Classe com métodos para configurar a Jetson Orin nano:
class configJetson {
private:
    // Identificador do controlador GPIO no JetPack 6
    const std::string chipIO = "gpiochip0";

    // Inicializa as linhas do GPIO da Jetson relativas aso pinos do barramento conector J12:
    struct LineJetson {
        const int line_PWM_A = 43;  // PH.00  - Pino 33 (Controle PWM laser)
        const int line_PWM_B = 41;  // PG.06  - Pino 32 (Controle strobo Led)

        const int line_IO_A = 136;  // PR.08  - Pino 8 (Controle Motor_01)
        const int line_IO_B = 135; //122;  // PY.00  - Pino 13 (Controle Motor_02)

        const int line_IO_C = 111;   // PY.02  - Pino  (Encoder_01)
        const int line_IO_D = 112;  // PI.01  - Pino  (Encoder_02)  
        const int line_IO_E = 50;  // PY.02  - Pino  (Encoder_03)
        const int line_IO_F = 122;   // PI.01  - Pino  (Encoder_04)

        const int line_IO_G = 144;  // PAC.06 - Pino 7  (Trigger camera de eventos)
        const int line_IO_H = 106;  // PQ.06  - Pino 31 (Trigger camera convencional)
        const int line_IO_I = 53;  // PY.04  - Pino 16 (Pisca Led)  
    };

    /*
    Parei aqui!! estava verificando a listagem dos pinos de IO desponíveis depois de configurar o PWM usando a classe class PWMLaser craida por mim.
    Até aqui compilou ok!!!!
    Proximos passos:
    1- Reservar os pinos de PWM 
    2- Chamar as funções, métodos, para ativar o PWM em main()
    3- Providenciar alterar pelo menu os valores percentuais do PWM.  
    */

    // Pinos fisicos do barramento  GPIO J12 da Jetson Orin nano:
    struct PinoJetson {
        const int header_pin_PWM_A = 33; // Controle laser (Também PWM)
        const int header_pin_PWM_B = 32; // Controle strobo Led (Também PWM)

        const int header_pin_IO_A = 18; // Controle Motor_01 
        const int header_pin_IO_B = 19;//13; // Controle Motor_02   

        const int header_pin_IO_C = 10; // Controle Encoder_01
        const int header_pin_IO_D = 11; // Controle Encoder_02   
        const int header_pin_IO_E = 12; // Controle Encoder_03
        const int header_pin_IO_F = 13; // Controle Encoder_04    

        const int header_pin_IO_G = 7;  // Trigger camera de eventos   
        const int header_pin_IO_H = 31; // Trigger camera convencional
        const int header_pin_IO_I = 35; // Pisca Led 
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
    
    // Guarda o pino do barramento da Jetson
    struct gpiod_line *line;  

    // Armazena a duração internamente
    int pulse_duration_ms; 

    // Método "run_blink" é a callback chamada pela Thread "blink_thread" para piscar o Led.    
    void run_blink();

public:
    // Construtor configurando pino e tempo:
    LEDController(struct gpiod_line *gpio_line, int duration_ms) : line(gpio_line), pulse_duration_ms(duration_ms) {}

    // Destrutor do objeto:
    ~LEDController() {
        stop();
    }


    // Inicia thread para piscar Led
    void start() {
        if (is_running) {
            std::cout << "Pulso Led ativado já está ativado.\n";
            return;
        }
        is_running = true;

        // Abre a thread "blink_thread" para piscar o led através do método, callback, "run_blink": 
        blink_thread = std::thread(&LEDController::run_blink, this);
    }


    //  Para execução da thread que pisca o Led
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



// Classe para ocntrole do PWM da Jetson usando os recursos do próprio Kernel do Linux usando a 
// interface Sysfs (Virtual files Systems). 
// 
class PWMLaser {

    // Na Jetson cada gerador de PWM é uma unidade de hardware independente.
    // Ela possui quatro chips de PWM, os dois utilziados são:
    // 3280000.pwm - pwmchip0 - PWM0 - Pino 32
    // 32c0000.pwm - pwmchip1 - PWM2 - Pino 33
    // 32a0000.pwm - pwmchipX - FAN  (ventoinha) - interno
    // 32e0000.pwm - pwmchipX - PWM3 - Pino 15
    // É pwmchipx porque pode variar a numeração. Para saber executar o comadno: 
    // $ sudo cat /sys/kernel/debug/pwm

private:
    std::string fullPath = " ";

    // Este método privado configura o chip com uma frequencia baseada no time passado em value_time_ns, (f= 1/value_time_ns):
    void writeToFile(std::string file, std::string value_time_ns) {
        std::ofstream fs(fullPath + file);
        if (fs.is_open()) {
            fs << value_time_ns;
            fs.close();
        }
    }

public:

    // Configura com freqquencia definida pelo parâmetro "periodo_ns", padrão é 1kHz (1.000.000 X 10^(-9)).
    void inicializar(std::string channel, long periodo_ns) {
        // Exporta o canal se necessário:
        std::ofstream export_file(channel);

        if (export_file.is_open()) { 
            export_file << "0"; export_file.close(); 
        }
        setPeriodo(periodo_ns);
    }

    // Seta a frequencia de rtabalho do PWM, padrão é 1kHz:
    void setPeriodo(long ns) { 
        writeToFile("period", std::to_string(ns)); 
    }
    
    // Ajusta o duty cicle com valor em nano segundos:
    void setDutyCycle(long ns) { 
        writeToFile("duty_cycle", std::to_string(ns)); 
    }

    // Habilita o PWM:
    void enable(bool state) { 
        writeToFile("enable", state ? "1" : "0"); 
    }

    // INciailiza o path do chip referente ao PWM usado 0 ou 1:
    void setPathFileChip(std::string path) { 
        fullPath= path; 
    }
   
};