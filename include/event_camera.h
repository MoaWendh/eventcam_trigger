#ifndef CAMERA_DE_EVENTOS
#define CAMERA_DE_EVENTOS

#include <string>
#include <iostream>

#include <metavision/sdk/stream/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>


struct paramsEventCam
{
    std::string serial_cam= "unknow";
    std::string versionFirm= "unknow";
    std::string dataEncodeFormat= "unknow";
    std::string plugin= "unknow";
    std::string fabricante= "unknow";
};


class EventCamera {
private:
    std::string serialNumber;
    // Variáveis que guardam os parametros gerais da camera:

    // O objeto real da SDK agora mora aqui dentro
    Metavision::Camera cam;

    // Ponteiro para a interface de Trigger de hardware
    Metavision::I_TriggerIn* trigger_in = nullptr;

    // Variavel tipo struct que guarda os paramerto gerais da camera
    paramsEventCam parametrosGerais;

    // Apenas guarda o estado aual da camera instanciada:
    bool inicializada;

    // Patha para leitura do json:
    std::string fileName= "settings.json";
    std::string path= "../";
    const std::string fullPath= path + '/' + fileName;

    // Variáveis que guardam a geometria da camera:
    int width = 0;
    int height = 0;

public:
    // Construtor que recebe o Serial Number:
    EventCamera(std::string serial);
    ~EventCamera(){};

    // 
    void readCameraBiases();

    // Inicializa a camera de eventos:
    bool openEventCam();

    // Método para leitura dos parêmtros gerias da camera de eventos:
    void getParametrosGeraisEventCam();

    // APenas para pegar o nº de serie:
    std::string getSerial(); 

    // Método que efetua a leitura dos biases e armazena em vairável:
    bool setBias();

    // Método que efetua a leitura das dimensões do sensor da camera de eventos, largura:
    int getWidth() { 
        auto &geo = cam.geometry(); 
        this->width = geo.get_width();        
        return width; 
    };
    
    // Método que efetua a leitura das dimensões do sensor da camera de eventos, altura:
    int getHeight() { 
        auto &geo = cam.geometry(); 
        this->height = geo.get_height();
        return height;
    };

    // Método que inicia a gravação, captura, dos eventos por trigger:
    bool startRecording(const std::string& fullPath);

    // Metodo que para a gravação dos eventos:
    bool stopRecording();

    // 
    void setCDCallback(std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)> cb) {
        cam.cd().add_callback(cb);
    }

    // Método para configurar o sincronismo de hardware, e gerar trigger por hardware:
    bool enableHardwareTrigger();

    // Método para iniciar a captura de eventos no hardware
    bool start();

    // Método para parar a captura de eventos no hardware
    void stop();
};

#endif