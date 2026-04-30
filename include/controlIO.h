#include <string>
#include <gpiod.h>
#include <atomic>
#include <thread>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <filesystem>


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
        const int line_PWM_B =41;   // PG.06  - Pino 32 (Controle strobo Led)

        const int line_IO_A = 112;  // PR.04  - Pino 11 (Controle Motor_01)
        const int line_IO_B = 122;  // PY.00  - Pino 13 (Controle Motor_02)

        const int line_IO_C = 53;   // PI.02  - Pino  (Encoder_01)
        const int line_IO_D = 113;  // PR.05  - Pino  (Encoder_02)  
        const int line_IO_E = 124;  // PY.02  - Pino  (Encoder_03)
        const int line_IO_F = 52;   // PI.01  - Pino  (Encoder_04)

        const int line_IO_G = 144;  // PAC.06 - Pino 7  (Trigger camera de eventos)
        const int line_IO_H = 106;  // PQ.06  - Pino 31 (Trigger camera convencional)
        const int line_IO_I = 51;   // PI.00  - Pino 40 (Pisca Led)  
    };


    // Pinos fisicos do barramento  GPIO J12 da Jetson Orin nano:
    struct PinoJetson {
        const int header_pin_PWM_A = 33; // Controle laser (Também PWM)
        const int header_pin_PWM_B = 32; // Controle strobo Led (Também PWM)

        const int header_pin_IO_A = 11; // Controle Motor_01 
        const int header_pin_IO_B = 13; // Controle Motor_02   

        const int header_pin_IO_C = 35; // Controle Encoder_01
        const int header_pin_IO_D = 36; // Controle Encoder_02   
        const int header_pin_IO_E = 37; // Controle Encoder_03
        const int header_pin_IO_F = 38; // Controle Encoder_04    

        const int header_pin_IO_G = 7;  // Trigger camera de eventos   
        const int header_pin_IO_H = 31; // Trigger camera convencional
        const int header_pin_IO_I = 40; // Pisca Led 
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



// Classe relacionada ao contrle de estado de Leds ou Lasers:
class LightController {
private:
    // Preferencialmente usar uma variável atômica em vez de primitiva, pois variáveis 
    // atomicas são mais adequadas para utilização com trheads, pois garantem sincornismo. 
    std::atomic<bool> is_active{false};

public:
    // 
    void setRunning(bool status) {
        is_active = status;
    }

    // 
    bool getRunning() const {
        return is_active.load(); // 
    };
};


// Classe para ocntrole do PWM da Jetson usando os recursos do próprio Kernel do Linux usando a 
// interface Sysfs (Virtual files Systems).
// Na Jetson cada gerador de PWM é uma unidade de hardware independente.
// Ela disponibiza quatro chips de PWM:
// 3280000.pwm - pwmchip0 - PWM0 - Geral/LCD
// 32a0000.pwm - pwmchip1 - PWM5 - Geral
// 32c0000.pwm - pwmchip2 - PWM1 - Pino 33 - 
// 32e0000.pwm - pwmchip3 - PWM7 - Pino 32 - Usado para controle potencia do laser
// É pwmchipx pode variar a numeração. Para saber executar o comadno: 
// $ sudo cat /sys/kernel/debug/pwm 

class PWM {
private:
    int export_pwm= 0; // Valor a zer 
    long period_pwm= 1000000; // Período do pwm em nano segundos. Inicia o pwm com T= 1.000.000 (frequencia de 1kHz), que é o default da Jetson, mais estável.
    long dutyCicle_pwm= 50; // Dutycicle em percetual (%). Inicia com 50%.
    bool active_pwm= false; //Quando este valor for true o pulso pwm será liberado na saída.
    std::string fullPath_pwm_chip = " ";
    std::string canal_pwm = "pwm0/";


    // Este método privado é usado para setar os parâmetros nos respectivos arquivos: periodo, duty-cicle e enable:
    bool writeToFile(std::string file, std::string value) {
        std::ofstream fs(fullPath_pwm_chip + canal_pwm + file);
        bool write_ok;
        if (fs.is_open()) {
            fs << value;
            fs.close();
            write_ok= true;
        }
        else
            write_ok= false;
        
        return write_ok; 
    }

public:
     // Seta a variável membro "periodo_pwm", que define a frequencia de trabalho do PWM, padrão é 1kHz:
    void setPeriodo(long periodo_ns) { 
        // Atualiza  variável que guarda o periodo:
        period_pwm= periodo_ns;

        // Configura o periodo:
        if (writeToFile("period", std::to_string(period_pwm)))
            std::cout << "Periodo PWM=: "<< period_pwm << std::endl;
        else
            std::cout << "[Erro] Não foi possível ajustar periodo pwm." << std::endl;
    }
    
    // Seta a variável membro "DutyCicle_pwm", que define a frequencia de trabalho do PWM, padrão é 1kHz:
    void setDutyCycle(long dutyCycle_ns) {
        dutyCicle_pwm= dutyCycle_ns;

        // Configura o duty_cicle
        long dutyCicle_pwm_ns= (dutyCicle_pwm*period_pwm)/100;
        if (writeToFile("duty_cycle", std::to_string(dutyCicle_pwm_ns)))
            std::cout << "Duty-Cicle PWM=: "<< dutyCicle_pwm << std::endl;
        else
            std::cout << "[Erro] Não foi possível ajustar o duty-cicle do pwm." << std::endl;         
    }

    // Seta a variável membro "active_pwm", usada para guardar o status de habilitação do pwm.
    // Também habilita a geração do sinal pwm: 
    bool enable() { 
        if (writeToFile("enable", "1"))
            active_pwm = true;
        else
            active_pwm = false;
        return active_pwm;   
    }


     // Desabilita o PWM:
    bool disable() { 
        if (writeToFile("enable", "0"))
            active_pwm = false;
        return active_pwm;
    }
   

    // INicializa o path do chip referente ao PWM:
    void setPathFileChip(std::string path) { 
        fullPath_pwm_chip= path; 
    }
   
    void setChannel(std::string channel){     
        canal_pwm= channel;    
    }

    // Metodo get para o duty-cicle:
    long getDutyCicle(){     
        return dutyCicle_pwm;    
    }



   // Inicializa o canal pwm:
    void inicializa_canal() {
        // Exporta o canal:

        // Primeiro verifica se a pasta pwm0 existe se sim o canal já foi exportado.
        // A pasta pwm0 é criada com o export, se ela já existe não tem porque recriá-la, basta recofnigurar o pwm 
        if (!std::filesystem::exists(fullPath_pwm_chip + "pwm0/")){
            // Se não existe exporta:
            std::ofstream export_file(fullPath_pwm_chip + "export");
            export_file << "0"; 
            export_file.close(); 
        }

        // Função "usleep()" serve para dar um tempo para que o hardware seja aajustado, sem um sleep de 1m não consegue setar o hw: 
        //usleep(100000);
        
        // Configura o periodo:
        //writeToFile("period", std::to_string(period_pwm));
        
        //usleep(100000);

        // Configura o duty_cicle
        //long dutyCicle_pwm_ns= (dutyCicle_pwm*period_pwm)/100;
       // writeToFile("duty_cycle", std::to_string(dutyCicle_pwm_ns)); 

        //usleep(100000);

        // Habilita o chip a gerar o pulso pwm:
        //enable();
    }

};