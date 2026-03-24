#include "controlJetson.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <gpiod.h>


// Método para efetuar a configuração do GPIO da Jetson
GPIO_Lines configJetson::configura_GPIO_Jetson(struct gpiod_chip **chip_ptr) {
    GPIO_Lines lines_out = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    //********************* Inicia configuração do barramenteo IO da Jetson: 
    // 1º. Abre o chip definido na string chipIO (privada da classe) e armazena no ponteiro fornecido pelo main:
    *chip_ptr = gpiod_chip_open_by_name(chipIO.c_str());

    if (!(*chip_ptr)) {
        perror("Erro ao abrir gpiochip0!!!");
        return lines_out;
    }

    // 2º. Captura o pino, line, para o TRIGGER da camera de eventos usando lineF (Pino 7):
    lines_out.triggerEventCam = gpiod_chip_get_line(*chip_ptr, lines.lineF);
    if (!lines_out.triggerEventCam) {
        std::cerr << "Erro: Nao foi possivel obter a linha: " << lines.lineF << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.triggerEventCam, 0);

    // 3º. Captura o pino, line, para piscar o LED usando lineA (Pino 33):
    lines_out.piscaLed = gpiod_chip_get_line(*chip_ptr, lines.lineA);
    if (!lines_out.piscaLed) {
        std::cerr << "Erro: Nao foi possivel obter a linha: " << lines.lineA << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.piscaLed, 0);    

    
    // 4º. Faz um request da linha como SAÍDA para o trigger:
    if (gpiod_line_request_output(lines_out.triggerEventCam, "sync_trigger", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.triggerEventCam, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Usa o mapeamento físico privado da classe para o log:
        std::cout << "Jetson: trigger no pino: " << pinos.header_pinF << std::endl;
        std::cout << "Jetson: Estado inicial do pino " << pinos.header_pinF << "= 0V" << std::endl;
    } 
    else {
        perror("Erro ao configurar saída");
        return lines_out;
    }

    // 5º. Faz um request da linha como SAÍDA para o led:
    if (gpiod_line_request_output(lines_out.piscaLed, "status_led", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.piscaLed, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "Jetson: Led no pino: " << pinos.header_pinA << std::endl;
        std::cout << "Jetson: Estado inicial do pino do led " << pinos.header_pinA << "= 0V" << std::endl;   
    } 
    else {
        perror("Erro ao configurar saída");
        return lines_out;
    }   

    return lines_out;

}


// Mpetodo que apeans fecha e libera o GPIO da Jetson:
void configJetson::liberaGPIO_Jetson(struct gpiod_chip *chip, GPIO_Lines gpios){

    // Libera as linhas individuais (se existirem)
    if (gpios.triggerEventCam) {
        gpiod_line_release(gpios.triggerEventCam);
        std::cout << "Jetson: Linha do trigger da camera de eventos liberada." << std::endl;
    }

    if (gpios.piscaLed) {
        gpiod_line_release(gpios.piscaLed);
        std::cout << "Jetson: Linha do LED liberada." << std::endl;
    }

     if (gpios.triggerNormalCam) {
        gpiod_line_release(gpios.triggerNormalCam);
        std::cout << "Jetson: Linha do trigger da camera convencional liberada." << std::endl;
    }

    if (gpios.controlLaser) {
        gpiod_line_release(gpios.controlLaser);
        std::cout << "Jetson: Linha do Laser liberada." << std::endl;
    }

     if (gpios.controlMotor01) {
        gpiod_line_release(gpios.controlMotor01);
        std::cout << "Jetson: Linha 1 do motor liberada." << std::endl;
    }   
 
      if (gpios.controlMotor02) {
        gpiod_line_release(gpios.controlMotor02);
        std::cout << "Jetson: Linha 2 do motor liberada." << std::endl;
    }     

     if (gpios.controlMotor03) {
        gpiod_line_release(gpios.controlMotor03);
        std::cout << "Jetson: Linha 3 do motor liberada." << std::endl;
    }  

     if (gpios.controlMotor04) {
        gpiod_line_release(gpios.controlMotor04);
        std::cout << "Jetson: Linha 4 do motor liberada." << std::endl;
    }  

    // Fecha o controlador (chip)
    if (chip) {
        gpiod_chip_close(chip);
        std::cout << "Jetson: Controlador " << chipIO << " fechado com sucesso." << std::endl;
    }
}