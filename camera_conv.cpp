#include "camera_conv.h"
#include "Spinnaker.h"
#include <string>


// O construtor da classe camera_conv:
CameraConv::CameraConv(std::string serial) 
    : serialNumber(serial), inicializada(false) {

    // Inicializa a camada de transporte e os recursos de sistema do SDK, vinculando a instância 
    // única (Singleton) ao ponteiro inteligente "system" para o controle de hardware:        
    system = Spinnaker::System::GetInstance();
}


// Destruidor da classe camera_conv:
CameraConv::~CameraConv() {
    close();
}

bool CameraConv::open() {
    try {
        Spinnaker::CameraList camList = system->GetCameras();
        
        // Busca a câmera específica pelo Serial Number
        pCam = camList.GetBySerial(serialNumber);

        if (!pCam.IsValid()) {
            std::cerr << "Câmera com Serial [" << serialNumber << "] não encontrada!" << std::endl;
            camList.Clear();
            return false;
        }

        // Estabelece a conexão real com o hardware
        pCam->Init();
        inicializada = true;

        // Limpa a lista, mas pCam mantém a referência para a câmera aberta
        camList.Clear();
        return true;
    }
    catch (const Spinnaker::Exception& e) {
        std::cerr << "Erro ao abrir câmera " << serialNumber << ": " << e.what() << std::endl;
        return false;
    }
}

void CameraConv::close() {
    try {
        if (inicializada && pCam.IsValid()) {
            pCam->DeInit();
            pCam = nullptr; // Libera o ponteiro inteligente
        }
        
        if (system.IsValid()) {
            // Nota: Só chame ReleaseInstance se tiver certeza que 
            // nenhuma outra instância de camera_conv está rodando.
            // Para sistema estéreo, o ideal é gerenciar isso no main.
            system->ReleaseInstance();
        }
        inicializada = false;
    }
    catch (const Spinnaker::Exception& e) {
        std::cerr << "Erro ao fechar câmera: " << e.what() << std::endl;
    }
}

void CameraConv::exibir_configuracao() {
    if (!inicializada) {
        std::cout << "Câmera não inicializada." << std::endl;
        return;
    }

    Spinnaker::GenApi::INodeMap& nodeMap = pCam->GetNodeMap();
    
    // Exemplo de leitura de um nó (Modelo)
    Spinnaker::GenApi::CStringPtr ptrModel = nodeMap.GetNode("DeviceModelName");
    if (Spinnaker::GenApi::IsAvailable(ptrModel)) {
        std::cout << "Câmera: " << ptrModel->GetValue() << " [Serial: " << serialNumber << "]" << std::endl;
    }
}

std::string CameraConv::get_serial() const {
    return serialNumber;
}

bool CameraConv::is_ok() const {
    return inicializada;
}

Spinnaker::GenApi::INodeMap& CameraConv::get_nodemap() {
    if (!inicializada) {
        throw std::runtime_error("Tentativa de acessar NodeMap de câmera não inicializada!");
    }
    return pCam->GetNodeMap();
}