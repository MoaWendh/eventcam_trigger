#include "controlIO.h"
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

    // Testa abertura do chip:
    if (!(*chip_ptr)) {
        perror("Erro ao abrir gpiochip0!!!");
        return lines_out;
    }

    // 2º. Captura os pinos, lines, no GPIO da Jetosn para todos os pinos que serão utilziados:
    // Para o TRIGGER da camera de eventos usando lineF (Pino 7):
    lines_out.triggerEventCam = gpiod_chip_get_line(*chip_ptr, lines.lineF);
    if (!lines_out.triggerEventCam) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do trigger da camera de eventos na Linha: " << lines.lineF << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.triggerEventCam, 0);

    // Para piscar o LED usando lineA (Pino 33):
    lines_out.piscaLed = gpiod_chip_get_line(*chip_ptr, lines.lineA);
    if (!lines_out.piscaLed) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do Led na linha: " << lines.lineA << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.piscaLed, 0);    


    // Para o Trigger da câmera convencional:
    lines_out.triggerNormalCam = gpiod_chip_get_line(*chip_ptr, lines.lineG);
    if (!lines_out.triggerNormalCam) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do trigger da camera convencional na linha: " << lines.lineG << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.triggerNormalCam, 0);    
    

    // Para o controle da fonte laser:
    lines_out.controlLaser = gpiod_chip_get_line(*chip_ptr, lines.lineH);
    if (!lines_out.controlLaser) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle do Laser na linha: " << lines.lineH << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.controlLaser, 0);     


     // Para o controle da fase 1 do motor:
    lines_out.controlMotor01 = gpiod_chip_get_line(*chip_ptr, lines.lineB);
    if (!lines_out.controlMotor01) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle da fase 1 do Motor na linha: " << lines.lineB << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.controlMotor01, 0);  
    
     // Para o controle da fase 2 do motor:
    lines_out.controlMotor02 = gpiod_chip_get_line(*chip_ptr, lines.lineC);
    if (!lines_out.controlMotor02) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle da fase 2 do Motor na linha: " << lines.lineC << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.controlMotor02, 0);   
    
    
     // Para o controle da fase 3 do motor:
    lines_out.controlMotor03 = gpiod_chip_get_line(*chip_ptr, lines.lineD);
    if (!lines_out.controlMotor03) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle da fase 3 do Motor na linha: " << lines.lineD << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.controlMotor03, 0); 
    
    // Para o controle da fase 4 do motor:
    lines_out.controlMotor04 = gpiod_chip_get_line(*chip_ptr, lines.lineE);
    if (!lines_out.controlMotor04) {
        std::cerr << "Erro: Nao foi possivel obter o pino para controle da fase 4 do Motor na linha: " << lines.lineE << std::endl;
        return lines_out;
    }
    gpiod_line_set_value(lines_out.controlMotor04, 0);     


    // 3º. Faz um request da linha como SAÍDA para o trigger:
    // Request do trigger da camera:
    if (gpiod_line_request_output(lines_out.triggerEventCam, "triggerEventCam", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.triggerEventCam, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // Usa o mapeamento físico privado da classe para o log:
        std::cout << "Jetson: trigger EvCam no pino: " << pinos.header_pinF << " (Nivel= 0V)"<< std::endl;
    } 
    else {
        perror("[ERRO de request] Não foi possível configurar como OUTPUT o pino do trigger da camera de eventos.");
        return lines_out;
    }

    // Request do controloe de piscagem do led:
    if (gpiod_line_request_output(lines_out.piscaLed, "piscaLed", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.piscaLed, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Pisca Led no pino: " << pinos.header_pinA << " (Nivel= 0V)" << std::endl;
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino do LED.");
        return lines_out;
    }   


    // Request do controloe de trigger da camera convencional:
    if (gpiod_line_request_output(lines_out.triggerNormalCam, "TriggerNOrmalCam", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.triggerNormalCam, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Trigger da camera convencional no pino: " << pinos.header_pinG << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de Trigger da camera convencional.");
        return lines_out;
    }   


     // Request do controle do LASER:
    if (gpiod_line_request_output(lines_out.controlLaser, "ControlLaser", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.controlLaser, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Controle do LASER no pino: " << pinos.header_pinH << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de controle do LASER.");
        return lines_out;
    }     


     // Request do controle da fase 1 do Motor:
    if (gpiod_line_request_output(lines_out.controlMotor01, "ControlMotor_Fase01", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.controlMotor01, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Controle MOTOR Fase 1 no pino: " << pinos.header_pinB << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de controle da Fase 01 do MOTOR.");
        return lines_out;
    }    


     // Request do controle da fase 2 do Motor:
    if (gpiod_line_request_output(lines_out.controlMotor02, "ControlMotor_Fase02", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.controlMotor02, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Controle MOTOR Fase 2 no pino: " << pinos.header_pinC << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de controle da Fase 02 do MOTOR.");
        return lines_out;
    }        


     // Request do controle da fase 3 do Motor:
    if (gpiod_line_request_output(lines_out.controlMotor03, "ControlMotor_Fase03", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.controlMotor03, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Controle MOTOR Fase 3 no pino: " << pinos.header_pinD << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de controle da Fase 03 do MOTOR.");
        return lines_out;
    }          


     // Request do controle da fase 4 do Motor:
    if (gpiod_line_request_output(lines_out.controlMotor04, "ControlMotor_Fase04", 0) == 0){
        // Garantir que o pino do IO da Jetson inicie em nivel baixo:
        gpiod_line_set_value(lines_out.controlMotor04, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Controle MOTOR Fase 4 no pino: " << pinos.header_pinE << " (Nivel= 0V)"<< std::endl; 
    } 
    else {
        perror("[ERRO de Request] Não foi possível configurar como OUTPUT o pino de controle da Fase 04 do MOTOR.");
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

// Método para piscar o led:
void LEDController::run_blink() {
    std::cout << "[Thread] Pulso do Led iniciada (" << pulse_duration_ms << "ms).\n";
    while (is_running) {
        //Ativa o pino para acender o led
        gpiod_line_set_value(line, 1);
        // Matém o lde acesso por pulse_duration_ms
        std::this_thread::sleep_for(std::chrono::milliseconds(pulse_duration_ms));

        // Apaga o Led:
        gpiod_line_set_value(line, 0);
        // Matme o led apagado por pulse_duration_ms:
        std::this_thread::sleep_for(std::chrono::milliseconds(pulse_duration_ms));
    }
    std::cout << "[Thread] Pulso finalizado.\n";
    // Por garantia:
    gpiod_line_set_value(line, 0);
}