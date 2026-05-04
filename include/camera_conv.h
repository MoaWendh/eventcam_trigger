#ifndef CAMERA_CONV_H
#define CAMERA_CONV_H

#include "Spinnaker.h"
#include <string>
#include <iostream>

class CameraConv {
private:
    // Objeto syste unico do tipo Singleton da classe Spinnaker::SystemPtr:
    Spinnaker::SystemPtr system;
    
    // "pCam" é um ponteiro inteligente que atua como a interface de controle direto da câmera. 
    // Ele encapsula o acesso aos registros de hardware (NodeMap), gerencia o fluxo de 
    // aquisição de imagens e garante a persistência da conexão com o dispositivo enquanto o objeto estiver aberto.
    Spinnaker::CameraPtr pCam;
    
    // Guarda o numero de série da camera referent a este objeto:
    std::string serialNumber;

    // Apenas guarda o estado aual da camera instanciada:
    bool inicializada;

public:
    // Construtor que recebe o Serial Number:
    CameraConv(std::string serial);
    ~CameraConv();

    // Métodos principais:
    bool open();
    void close();
    void exibir_configuracao();

    // Getters:
    std::string get_serial() const;
    bool is_ok() const;
    
    // Método para obter o NodeMap (necessário para triggers e parâmetros)
    Spinnaker::GenApi::INodeMap& get_nodemap();
};

#endif