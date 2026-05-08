#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/device/device_discovery.h>

#include "event_camera.h"

using json = nlohmann::json;

// O construtor da classe camera_conv:
EventCamera::EventCamera(std::string serial) 
    : serialNumber(serial), inicializada(false) {
}

// Esta função tenta abrir a câmera de eventos usando o serial number fornecido no construtor. 
// Se falhar, ela tenta listar todas as câmeras conectadas para ajudar na depuração. 
bool EventCamera::openEventCam(){
    try {
            // Forma alternativa de abrir a câmera, sem usar o serial number, mas ela pode abrir a câmera errada se houver mais de uma conectada:
            //camera = Metavision::Camera::from_first_available();

            // Se a câmera for encontrada e aberta com sucesso, o objeto cam é inicializado com a câmera correspondente ao serial number fornecido.
            // Isto singifica que após gerar o objeto cam, a câmera de eventos já está aberta e pronta para uso.
            // Estabeleceu um canal de comunicação: agora o objeto cam é a interface para interagir com a câmera, ler eventos, configurar parâmetros, etc.
            // Hardware incializado: agora é possível acessar as funcionalidades do device câmera através do objeto cam.
            cam = Metavision::Camera::from_serial(serialNumber);
            return true;
        } 
        catch (const Metavision::CameraException &e) {
            std::cerr << "Erro ao abrir a camera: " << e.what() << std::endl;
            try{
                // Tenta capturar a lista de seriais de todas as câmeras conectadas
                Metavision::DeviceDiscovery::SerialList dispositivos= Metavision::DeviceDiscovery::list();
                if (!dispositivos.empty()){
                    int numDevices= dispositivos.size();
                    std::cout << "Cameras conectadas:" << std::endl;
                    for (int ct=0; ct<numDevices; ct++){
                        // std::next(iterador_inicial, n_posições)
                        auto it = std::next(dispositivos.begin(), ct);
                        std::string sn = *it; 
                        
                        std::cout << "Câmera [" << ct << "] - Serial: " << sn << std::endl;
                    }
                }
                else
                    std::cerr << "Nenhuma câmera de eventos detectada no barramento USB." << std::endl; 
            }
            catch (const std::exception &e) {
                std::cerr << "[ERRO] Falha ao escanear barramento USB." << e.what() << std::endl;
            }    
            return false;
    }
}


void EventCamera::getParametrosGeraisEventCam(){
    // Captura dados da câmera instanciada:
    try {
        std::cout << "*** Camera de eventos ***"  << std::endl; 
        parametrosGerais.fabricante = cam.get_camera_configuration().integrator;

        std::cout << "Plugin versão........: " << parametrosGerais.fabricante << std::endl; 

        parametrosGerais.plugin = cam.get_camera_configuration().plugin_name;
        std::cout << "Plugin versão........: " << parametrosGerais.plugin << std::endl; 

        parametrosGerais.serial_cam = cam.get_camera_configuration().serial_number;
        std::cout << "Nº Serial cam........: " << parametrosGerais.serial_cam << std::endl;

        parametrosGerais.versionFirm = cam.get_camera_configuration().firmware_version;
        std::cout << "Versão do firmware...: " << parametrosGerais.versionFirm << std::endl;

        parametrosGerais.dataEncodeFormat = cam.get_camera_configuration().data_encoding_format;
        std::cout << "Formato dos dados....: " << parametrosGerais.dataEncodeFormat << std::endl;
        std::cout << std::endl;
    } 
    catch (...) {
        std::cout << "Nao foi possivel obter o serial via CameraConfiguration." << std::endl;
    }

}

// Função chamada para a leitura dois Biases da câmera de eventos:
void EventCamera::readCameraBiases() {
    try {
        // Acessar via HAL explicitamente:
        auto *biases = cam.get_device().get_facility<Metavision::I_LL_Biases>();

        if (biases) {
            std::cout << "\n--- Configuracao Atual de Biases (HD Sensor) ---" << std::endl;
            
            // Lista de biases comuns no IMX636 para verificar manualmente
            std::vector<std::string> bias_names = {
                 "bias_diff", "bias_diff_on", "bias_diff_off", "bias_fo", "bias_hpf", "bias_refr"
            };

            for (const auto& name : bias_names) {
                try {
                    int val = biases->get(name);
                    std::cout << " Bias: " << std::left << std::setw(15) << name 
                              << " | Valor: " << val << std::endl;
                } catch (...) {
                    // Se um nome específico não existir neste modelo, ignore
                }
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Erro: " << e.what() << std::endl;
    }    
}


// Função que busca os valores dos biases no arquivo .json com as confuguraçloes da câmera:
int lerJsonFile(std::string path, std::string biasName){
    std::ifstream file(path);
    if (!file.is_open()) 
        return -1;

    json data;
    file >> data;

    // Navega na estrutura: ll_biases_state -> bias (que é um array)
    auto biases = data["ll_biases_state"]["bias"];

    for (auto& item : biases) {
        if (item["name"] == biasName) {
            return item["value"];
        }
    }
    std::cout << "Erro!!! Campo de leitura: " << biasName << " não encontrado no arquivo: "<< path << std::endl;
    return -1; // Não encontrado
}



bool EventCamera::setBias() {

    // Os biases max e min. são definidos em https://docs.prophesee.ai/stable/hw/manuals/biases.html 
    // Os calores para a ca~mera SilkyEvCam pertencem a geração Gen3.1 VGA, assim os valores máximos e mínimo
    // são deinfidos como:
    
    const int bias_diff_default= 299; // Não alterar o valor do bias_diff, o default é 299.
    
    int bias_diff_on_min= bias_diff_default + 75; // O valor mínimo do bias_diff_on é bias_dif_default + 75.
    int bias_diff_on_max= bias_diff_default + 200; // O valor máximo do bias_diff_on é bias_dif_default + 200.
    
    int bias_diff_off_min= 100; // O valor mínimo do bias_diff_off é 100.
    int bias_diff_off_max= bias_diff_default - 65; // O valor máximo do bias_diff_off é bias_dif_default -65
    
    int bias_fo_min= 1250;
    int bias_fo_max= 1800;
    
    int bias_hpf_min= 900;
    int bias_hpf_max= 1800;
   
    int bias_refr_min= 1300;
    int bias_refr_max= 1800;

    // ANtes de gravar os valores de biases na camera verifica se os valores extrapolam os limites:
    int bias_diff_off= lerJsonFile(fullPath, "bias_diff_off");
    if (bias_diff_off<bias_diff_off_min || bias_diff_off>bias_diff_off_max){
        std::cout<< "ERRO!! O valor de bias_diff_off= "<< bias_diff_off << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_diff_off_min << " e " << bias_diff_off_max << std::endl;  
        return false;
    }

    int bias_diff_on= lerJsonFile(fullPath, "bias_diff_on");
    if (bias_diff_on<bias_diff_on_min || bias_diff_on>bias_diff_on_max){
        std::cout<< "ERRO!! O valor de bias_diff_on= "<< bias_diff_on << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_diff_on_min << " e " << bias_diff_on_max << std::endl;  
        return false;
    }

    int bias_fo= lerJsonFile(fullPath, "bias_fo");
    if (bias_fo<bias_fo_min || bias_fo>bias_fo_max){
        std::cout<< "ERRO!! O valor de bias_fo= "<< bias_fo << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_fo_min << " e " << bias_fo_max << std::endl;  
        return false;
    }   
    
    int bias_hpf= lerJsonFile(fullPath, "bias_hpf");
    if (bias_hpf<bias_hpf_min || bias_hpf>bias_hpf_max){
        std::cout<< "ERRO!! O valor de bias_hpf= "<< bias_hpf << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_hpf_min << " e " << bias_hpf_max << std::endl;
        return false;  
    }   

    int bias_refr= lerJsonFile(fullPath, "bias_refr");
    if (bias_refr<bias_refr_min || bias_refr>bias_refr_max){
        std::cout<< "ERRO!! O valor de bias_refr= "<< bias_refr << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_refr_min << " e " << bias_refr_max << std::endl; 
        return false; 
    }  


    // Acessa a Facility de Biases de baixo nível:
    auto *i_ll_biases = cam.get_device().get_facility<Metavision::I_LL_Biases>();
    // Testa se o acesso foi liberado:
    if (!i_ll_biases) {
        std::cerr << "[Erro] Nao foi possivel acessar a interface de Biases do hardware!" << std::endl;
        return false;
    }

    // Atualzia os biases na câmera:
    try {
        // Atuazia os valores um por um diretamente no registrador do sensor. Este processo é "on-the-fly", sem precisar de cam.stop()
        i_ll_biases->set("bias_diff_on", bias_diff_on);
        i_ll_biases->set("bias_diff_off", bias_diff_off);
        i_ll_biases->set("bias_fo", bias_fo);
        i_ll_biases->set("bias_hpf", bias_hpf);
        i_ll_biases->set("bias_refr", bias_refr);

        std::cout << "Biases atualizados com dados do arquivo \""<< fileName.c_str() << "\"" << " ...câmera: " << serialNumber << std::endl;
        return true;

    } catch (const std::exception &e) {
        std::cerr << "[Erro] Não foi possível gravar biases na câmera!!! " << e.what() << std::endl;
        return false;
    }   

    // Abaixo segue um método aleternativo para setar os biases da camera, ele é de mais alto nível, com apenas 1 linha:
    // cam.load(path.c_str());
    //return true;
}


// Este método configura a camera de eventos para gerar trigger por hardware:
void EventCamera::enableHardwareTrigger() {
    try {
        // O método get_device() retorna o dispositivo atual
        // O facility retorna um ponteiro para a interface de TriggerIn, que é a responsável por configurar o sincronismo de hardware e gerar trigger por hardware:
        auto *i_trigger_in = cam.get_device().get_facility<Metavision::I_TriggerIn>();

        if (i_trigger_in) {
            // Habilita o canal principal (Main) para bordas de subida e descida
            i_trigger_in->enable(Metavision::I_TriggerIn::Channel::Main);           
            std::cout << "Trigger habilitado - bordas subida e descida ..............câmera: " << serialNumber << std::endl;
        } 
        else 
            std::cerr << "[Erro] Não foi possível acessar a interface de TriggerIn do hardware!" << std::endl;
    }
    catch (const std::exception &e) {
        std::cerr << "[EXCECAO Trigger]: " << e.what() << std::endl;
    }    
}


bool EventCamera::start() {
    try {
        // Inicia a câmera interna da classe
        cam.start();      
        std::cout << "Captura de eventos iniciada................................câmera: " <<  serialNumber << std::endl;
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha ao iniciar a camera " << serialNumber 
                  << ": " << e.what() << std::endl;
        return false;
    }
}


void EventCamera::stop() {
    try {
        cam.stop();
        std::cout << "Captura e eventos finalizada...câmera: " << serialNumber << std::endl;
    } catch (...) {
        // Ignora erros ao parar para garantir que o programa feche
    }
}

// Este método inciai o resgistro dos eventos, solicitado pelo trigger de hardware: 
bool EventCamera::startRecording(const std::string& fullPath) {
    try {
        return cam.start_recording(fullPath);
    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha ao iniciar gravação: " << e.what() << std::endl;
        return false;
    }
}


// Para a gravação de eventos:
bool EventCamera::stopRecording() {
    return cam.stop_recording();
}


// Retorne o serial que já está na classe
std::string EventCamera::getSerial() {
    return serialNumber;
}