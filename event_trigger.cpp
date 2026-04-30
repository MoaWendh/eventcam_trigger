#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/device/device_discovery.h>
#include <vector>

#include "controlIO.h"

using json = nlohmann::json;

// Simples rotina para limpar a tela:
void limparTela() {
    // \033[2J: Limpa a tela inteira
    // \033[H: Move o cursor para a posição inicial (canto superior esquerdo)
    std::cout << "\033[2J\033[H" << std::flush;
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


// Função que gera trem de N pulsos, onde N é definido por numPulse:
void pulseTrigger(int numPulse, gpiod_line *line, int pin, int64_t duracaoPulso){
    std::cout << " Gerando pulso de: " << duracaoPulso << "ms no pino: "<< pin << std::endl;

    for (int ctPulse=0; ctPulse<numPulse; ctPulse++){
        gpiod_line_set_value(line, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(duracaoPulso));
        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(duracaoPulso));
    }
    std::cout << " Pulso finalizado." << std::endl;
}


// Funação que gera o menu de opções:
void showMenu(int pinTrigger, int pinLed, int64_t duracao){     
    std::cout << ""<< std::endl;
    std::cout << "******************** Menu  ********************"<< std::endl;
    std::cout << " 1 - Ler biases da câmera" << std::endl;
    std::cout << " 2 - Gravar biases na câmera" << std::endl;   
    std::cout << " 3 - Trigger: iniciar gravacao de eventos (.raw)" << std::endl;       
    std::cout << " 4 - Start/Stop blink Led"<< std::endl;
    std::cout << " > - Incrementa pulso Led" << std::endl;
    std::cout << " < - Decrementa pulso Led" << std::endl;
    std::cout << " + - Incrementa potência Led" << std::endl;
    std::cout << " - - Decrementa potência Led" << std::endl;
    std::cout << " L - Limpa Tela" << std::endl;
    std::cout << " Q - Sair do programa "<< std::endl;
    std::cout << "**********************************************"<< std::endl;
    std::cout << " Digite a opção: ";
    std::cout << std::endl;
}


// 
void saveDataFileWithTrigger(Metavision::Camera &cam, gpiod_line *line, int64_t duracao_trigger_microSeg, int64_t duracao_PreTrigger_microSeg, int64_t duracao_PosTrigger_microSeg, std::string serial) {
    try {
        // Obter data para o nome da pasta e do arquivo:
        auto agora = std::chrono::system_clock::now();
        auto tempo_t = std::chrono::system_clock::to_time_t(agora);
        struct tm *info = std::localtime(&tempo_t);

        // gera o noem do diretório:
        std::stringstream ss_pasta;
        ss_pasta << "data_evecam_" << std::put_time(info, "%d_%m_%Y");
        std::string nome_pasta = ss_pasta.str();

        // Criar a pasta se ela não existir
        if (!std::filesystem::exists(nome_pasta)) {
            std::filesystem::create_directory(nome_pasta);
        }

        // Gera o nomde do arquivo de dados .raw:
        std::stringstream ss_file;
        ss_file << "evecam_sn_" << serial << "_" << std::put_time(info, "%H%M%S") << ".raw";        
        std::string filename= ss_file.str();

        // Gera full path para salvar o arquivo de dados:
        std::string full_path = nome_pasta + "/" + filename;
        

        // Inicia Gravação doarquivo de dados:
        cam.start_recording(full_path);
        
        // Pré-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg para garantir que o arquivo foi aberto e o buffer inicializou:
        std::this_thread::sleep_for(std::chrono::microseconds(duracao_PreTrigger_microSeg));

        // *******************Início do pulso de trigger**************** 
        // Transição do trigger para nível alto:
        gpiod_line_set_value(line, 1);  

        // Mantém o pulso em nivel alto pelo tempo especificado em "duracao_trigger_microSeg":
        std::this_thread::sleep_for(std::chrono::microseconds(duracao_trigger_microSeg));

        // Transição do trigger para nível alto:
        gpiod_line_set_value(line, 0);
        // ********************Fim do pulso de trigger******************

        // Pós-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg antes de fecahr o arquivo de dados: 
        std::this_thread::sleep_for(std::chrono::microseconds(duracao_PosTrigger_microSeg));

        // Finaliza a Gravação e fecha arquivo de dados:
        if (cam.stop_recording()){
            std::cout << "Salvando dados ....." << std::endl;
            std::cout << "Dados salvos no arquivo: " << "\"" << filename << "\"" << std::endl;
        }
        else{
            std::cout << "!!!ERRO ao fechar arquivo:" << filename << std::endl;

        }

    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}



// Função chamada para a leitura dois Biases da câmera de eventos:
void readCameraBiases(Metavision::Camera &cam) {
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



// Função chamada para a leitura dois Biases da câmera de eventos:
int loadCameraBiases(Metavision::Camera &cam) {

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

    std::string fileName= "settings.json";

    std::string path= "../";

    std::string fullPath = path + '/' + fileName;

    // ANtes de gravar os valores de biases na camera verifica se os valores extrapolam os limites:
    int bias_diff_off= lerJsonFile(fullPath, "bias_diff_off");
    if (bias_diff_off<bias_diff_off_min || bias_diff_off>bias_diff_off_max){
        std::cout<< "ERRO!! O valor de bias_diff_off= "<< bias_diff_off << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_diff_off_min << " e " << bias_diff_off_max << std::endl;  
        return -1;
    }

    int bias_diff_on= lerJsonFile(fullPath, "bias_diff_on");
    if (bias_diff_on<bias_diff_on_min || bias_diff_on>bias_diff_on_max){
        std::cout<< "ERRO!! O valor de bias_diff_on= "<< bias_diff_on << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_diff_on_min << " e " << bias_diff_on_max << std::endl;  
        return -1;
    }

    int bias_fo= lerJsonFile(fullPath, "bias_fo");
    if (bias_fo<bias_fo_min || bias_fo>bias_fo_max){
        std::cout<< "ERRO!! O valor de bias_fo= "<< bias_fo << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_fo_min << " e " << bias_fo_max << std::endl;  
        return -1;
    }   
    
    int bias_hpf= lerJsonFile(fullPath, "bias_hpf");
    if (bias_hpf<bias_hpf_min || bias_hpf>bias_hpf_max){
        std::cout<< "ERRO!! O valor de bias_hpf= "<< bias_hpf << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_hpf_min << " e " << bias_hpf_max << std::endl;
        return -1;  
    }   

    int bias_refr= lerJsonFile(fullPath, "bias_refr");
    if (bias_refr<bias_refr_min || bias_refr>bias_refr_max){
        std::cout<< "ERRO!! O valor de bias_refr= "<< bias_refr << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_refr_min << " e " << bias_refr_max << std::endl; 
        return -1; 
    }  


    // Acessa a Facility de Biases de baixo nível:
    auto *i_ll_biases = cam.get_device().get_facility<Metavision::I_LL_Biases>();
    // Testa se o acesso foi liberado:
    if (!i_ll_biases) {
        std::cerr << "[Erro] Nao foi possivel acessar a interface de Biases do hardware!" << std::endl;
        return -1;
    }

    // Atualzia os biases na câmera:
    try {
        // Atuazia os valores um por um diretamente no registrador do sensor. Este processo é "on-the-fly", sem precisar de cam.stop()
        i_ll_biases->set("bias_diff_on", bias_diff_on);
        i_ll_biases->set("bias_diff_off", bias_diff_off);
        i_ll_biases->set("bias_fo", bias_fo);
        i_ll_biases->set("bias_hpf", bias_hpf);
        i_ll_biases->set("bias_refr", bias_refr);

        std::cout << "[OK] Biases atualizados na câmera com dados do arquivo: \""<< fileName.c_str() << "\"" << std::endl;
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[Erro] Não foi possível gravar biases na câmera!!! " << e.what() << std::endl;
        return -1;
    }   

    // Abaixo segue um método aleternativo para setar os biases da camera, ele é de mais alto nível, com apenas 1 linha:
    // cam.load(path.c_str());
    return 0;
}



void incrementaFreqLed(LightController& led, PWM& pwm_blink_led, bool use_led){
    if (use_led){
        if (!led.getRunning()){
            std::cout<<" Led está desativado"<<std::endl;
        }
        else{    
            int passo= 2;
            long dutyCicle= pwm_blink_led.getDutyCicle();
            if (dutyCicle<10){
                if (dutyCicle <= (100-passo))
                    dutyCicle += passo;
                else
                    std::cout<< "DutyCicle= 100%)"; 
                pwm_blink_led.setDutyCycle(dutyCicle);
            }  
            else
                std::cout<< "[Led LT2PR] Duty cicle atingiu o valor máximo de 10." <<std::endl;
        }
    }    
    else{
        int passo= 2;
        long dutyCicle= pwm_blink_led.getDutyCicle();

        if (dutyCicle <= (100-passo))
            dutyCicle += passo;
        else
            std::cout<< "DutyCicle= 100%)"; 
        pwm_blink_led.setDutyCycle(dutyCicle);
    }

}


// Funcção principal:
int main(int argc, char *argv[]) {
    // Rotina que limpa o terminal:
    limparTela();

    // Flag que habilita ou não o menu, apenas para testes:
    bool hab_exibe_menu= true;  
    
    // Flaque que inicializa o loop while:
    bool running = true; 

    // Definiçao dos números de série das câmeras de eventos:
    const std::string serialNumber_cam0= "00000414"; // HD
    const std::string serialNumber_cam2= "00000679"; // VGA
    const std::string serialNumber_cam3= "00000680"; // VGA

    //********************************************************************************************/
    //****************************** Configuração GPIO da Jetson *******************************/
    //********************************************************************************************/
    //Define duração de alguns pulsos:
    int64_t duracao_PulsoTrigger_microSeg= 30000; // em micro segundos
    int64_t duracao_PreTrigger_microSeg= 50000; // em micro segundos
    int64_t duracao_PosTrigger_microSeg= 50000; // em micro segundos
    int64_t duracao_PulsoLed_microSeg= 200000; // em micro segundos;
 
    // Declara o ponteiro do chip aqui para poder ser fechado dentro do main:
    struct gpiod_chip *chip = nullptr;

    // Isntancia um objeto para acessar a cofniguração do GPIO da Jetson. A classe está declarada no header "controlJetson.h":
    configJetson configuraGPIO_Jetson;

    // Chama o método get() para capturar as informações dos pinos e lines "ativos" da Jetson:    
    auto activePins= configuraGPIO_Jetson.getPinos();
    
    // Separa as informações dos pinos por função:
    int pin_PiscaLed= activePins.header_pin_IO_I;
    int pin_TriggerEventCam= activePins.header_pin_IO_G;
    int pin_TriggerCam= activePins.header_pin_IO_H;
    //int pin_ControlLaser= activePins.header_pinH;


    // Chama o método de configuração "configuraGPIO_Jetson.configura_GPIO_Jetson(&chip)" para inicialização do barramento GPIO da Jetson.
    // Ela retorna uma struct contendo ponteiros com os endereços de cada linha do GPIO da Jetson para controle pelo Kernell.
    // Através desses endereços que os pinos de IO são diretamente manipulados, por exemplo, mudanças de nivel lógico.
    GPIO_Lines gpios_actives= configuraGPIO_Jetson.configura_GPIO_Jetson(&chip);


    // Verificação se os pinos forma configurados ok, por segurança:
    if (gpios_actives.triggerEventCam == nullptr || gpios_actives.piscaLed == nullptr || gpios_actives.triggerNormalCam == nullptr || 
        gpios_actives.controlMotor01 == nullptr || gpios_actives.controlMotor02 == nullptr ) {
            std::cerr << "[ERRO] Falha crítica na inicialização dos GPIOs. Abortando!!!!" << std::endl;

        if (chip) 
            gpiod_chip_close(chip);
        return 1;
    }



    // ***********************************************************************************************/
    // ******************************** Configuraçaõ do PWM ******************************************/
    // ***********************************************************************************************/    
    // Configuração do PWM direta via Sysfs (Linux Kernel), que utiliza a interface de arquivos virtuais do 
    // sistema operacional para manipular registradores de hardware nativos. Este método elimina 
    // dependências externas, garantindo que o sinal de PWM seja gerado de forma estável por hardware 
    // independente, sem sobrecarga da CPU.

    // Define se está usando o led de potencia LT2PR da Opto Engineering
    bool useLed_LT2PR= true;

    std::string channelToExport_A= "/sys/class/pwm/pwmchip3/"; // 
    std::string channelToExport_B= "/sys/class/pwm/pwmchip2/"; // 

    // Instanciando o objeto controlLaser com a classe PWMLaser
    PWM pwm_BlinkLed;
    PWM pwm_PowerLed;

    // Deiniçãodo Duty-cicle:
    long dutyCicle_PWM_A= 1;  // Valor em nano segundos. PWM referente ao controle do duty cicle para blink led.
    long dutyCicle_PWM_B= 10;  // Valor em nano segundos. PWM referente ao controle da tensão.

    // Periodo dos PWMs, valor em nano segundos. 
    long periodo_PWM_A= 100000000;  // 100ms, valor em nano segundos. 
    //long periodo_PWM_B= 1000000;  // 1ms, valor em nano segundos. 
    long periodo_PWM_B= 1000000;  // 1ms, valor em nano segundos. 

    // Primeiro define o chip de trabalho com o path referente a este periférico:
    pwm_BlinkLed.setPathFileChip(channelToExport_A);
    pwm_PowerLed.setPathFileChip(channelToExport_B);

    // Inicializa o chip que controla o pwm:
    pwm_BlinkLed.inicializa_canal();
    usleep(100000);
    pwm_PowerLed.inicializa_canal();    
    usleep(100000);

    // Ajusta o período do pwm:
    pwm_BlinkLed.setPeriodo(periodo_PWM_A);
    usleep(100000);
    pwm_PowerLed.setPeriodo(periodo_PWM_B);
    usleep(100000);

    // Define o dutyciclo conforme o valor da variável timeDutyCicle_PWM_A:
    pwm_BlinkLed.setDutyCycle(dutyCicle_PWM_A);
    usleep(100000);
    pwm_PowerLed.setDutyCycle(dutyCicle_PWM_B);
    usleep(100000);

    // Inicaliza a geração do pulso pwm:
    // pwm_BlinkLed.enable();
    pwm_PowerLed.enable();

    // Define a variável que contém o tempo de duração que o LEd irá píscar:
    int duracao_PulsoLed_miliSeg= 20; // em micro segundos;
    

    // Instancia o objeto da classe LighCotroller: 
    LightController Led;

    //********************************************************************************************/
    //*************************** Configuração da camera de eventos ***********************/
    //********************************************************************************************/
    // Booleano que define se a câmera usada é HD ou VGA:    
    bool sensorHD= true;    

    // 1- Instancia um objeto "camera" da classe "Metavision::camera":
    Metavision::Camera camera;
    try {
        //camera = Metavision::Camera::from_first_available();
        //camera = Metavision::Camera::from_serial(serialNumber_cam0);
        camera = Metavision::Camera::from_serial(serialNumber_cam3);
    } 
    catch (const Metavision::CameraException &e) {
        std::cerr << "Erro ao abrir a camera: " << e.what() << std::endl;
        try{
            // Captura a lista de seriais de todas as câmeras conectadas
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
            else{
               std::cerr << "Nenhuma câmera de eventos detectada no barramento USB." << std::endl; 
            }
        }
        catch (const std::exception &e) {
            std::cerr << "[ERRO] Falha ao escanear barramento USB." << e.what() << std::endl;
        }
        return 1;
    }

    // Variáveis que guardam os dados da camera:
    std::string serial_cam_1= "unknow";
    std::string versionFirm= "unknow";
    std::string dataEncodeFormat= "unknow";
    std::string plugin= "unknow";
    std::string fabricante= "unknow";


    // Captura dados da câmera instanciada:
    try {
        fabricante = camera.get_camera_configuration().integrator;
        std::cout << std::endl;
        std::cout << "Plugin versão: " << fabricante << std::endl; 

        plugin = camera.get_camera_configuration().plugin_name;
        std::cout << "Plugin versão: " << plugin << std::endl; 

        serial_cam_1 = camera.get_camera_configuration().serial_number;
        std::cout << "Nº Serial cam: " << serial_cam_1 << std::endl;

        versionFirm = camera.get_camera_configuration().firmware_version;
        std::cout << "Versão do firmware: " << versionFirm << std::endl;

        dataEncodeFormat = camera.get_camera_configuration().data_encoding_format;
        std::cout << "Formato dos dados: " << dataEncodeFormat << std::endl;
        std::cout << std::endl;
    } 
    catch (...) {
        std::cout << "Nao foi possivel obter o serial via CameraConfiguration." << std::endl;
    }

    // Carregando os biases na camera de eventos definidos em setings.json:
    // camera.save("../settings.json");
    if (loadCameraBiases(camera))
        std::cout << "ERRO!! Biases não gravado na câmera." << std::endl;


    // Para transformas os evenvetos em imagens:
    // Obtém as dimensões do sensor:
    const auto &geometry = camera.geometry();
    
    // Cria um buffer de memória com as dimensões, geometria, obtidas acima.
    // Equivale a um tradutor de eventos para imagens:
    Metavision::CDFrameGenerator cd_frame_generator(geometry.get_width(), geometry.get_height());
    
    // Define o tempo de exposição dos eventos acumulados.
    // Aguarda acumular 10.000 us (10ms) de eventos para exibição: 
    cd_frame_generator.set_display_accumulation_time_us(10000);

    std::mutex cd_frame_mutex;
    cv::Mat cd_frame;
    std::atomic<bool> show_viewer{true}; 
    std::string window_name = "Visualizacao em Tempo Real";

    // Gerador de frames interno
    cd_frame_generator.start(30, [&](const Metavision::timestamp &ts, const cv::Mat &frame) {
        if (show_viewer) {
            std::unique_lock<std::mutex> lock(cd_frame_mutex);
            frame.copyTo(cd_frame);
        }
    });

    // Callback para processar eventos CD
    camera.cd().add_callback([&](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
        if (show_viewer) {
            cd_frame_generator.add_events(ev_begin, ev_end);
        }
    });


    // Habilita Trigger via HAL API
    auto *i_trigger_in = camera.get_device().get_facility<Metavision::I_TriggerIn>();
    if (i_trigger_in) {
        i_trigger_in->enable(Metavision::I_TriggerIn::Channel::Main);
        std::cout << std::endl;
        std::cout << "Trigger habilitado para ambas as bordas, Risinge e Falling edge." << std::endl;
    }

    // Start camera de eventos:
    camera.start();

    // Dummy Trigger: "Acorda" o canal de trigger da Metavision
    gpiod_line_set_value(gpios_actives.triggerEventCam, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpiod_line_set_value(gpios_actives.triggerEventCam, 0);

    // Aguarda o sistema processar esse evento interno antes de liberar o menu
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


    std::cout << "Camera inicializada" << std::endl;



    //********************************************************************************************  
    //****************************** Loop principal **********************************************
    //********************************************************************************************/    
    while(running) {

        int key = cv::waitKey(1);

        // Se o viewer estiver ativo, atualiza a janela sem travar o menu
        if (show_viewer) {
            std::unique_lock<std::mutex> lock(cd_frame_mutex);
            if (!cd_frame.empty()) {
                cv::imshow(window_name, cd_frame);
            }
           // cv::waitKey(1); 
        }    
        else {
                // Cria uma imagem preta pequena apenas para manter o foco do teclado
                cv::imshow(window_name, cv::Mat::zeros(100, 300, CV_8UC1));
            }            


        /// Exibe menu de escolha:
        if (hab_exibe_menu){
            //limparTela();
            showMenu(pin_TriggerEventCam, pin_PiscaLed, duracao_PulsoTrigger_microSeg);
            hab_exibe_menu= false;
        }


        // 2. Captura de Teclado via OpenCV (33ms de espera = ~30 FPS)
        // Isso substitui o std::cin e não trava o programa
        //int key = cv::waitKey(100);

        if (key!=-1){
            char my_char= static_cast<char>(key);

            switch (my_char)
            {
                case '1':
                        readCameraBiases(camera);
                        break;
                
                case '2':
                        // Por que a camera para quando eu chamo afunção loadBiasesNaCamera()?
                        //show_viewer = false;
                        //camera.stop();
                        loadCameraBiases(camera);
                        //camera.start();
                       // show_viewer = true;
                        break;

                case '3':
                        {
                            std::cout << "\nTrigger Com duração de: "<< duracao_PulsoTrigger_microSeg << "us" << std::endl;
            
                            // Passando o objeto camera, o ponteiro da linha GPIO e a duração do pulso trigger
                            std::thread t(saveDataFileWithTrigger, std::ref(camera), gpios_actives.triggerEventCam, duracao_PulsoTrigger_microSeg, duracao_PreTrigger_microSeg, duracao_PosTrigger_microSeg, serial_cam_1);
                            
                            // Desacoplar a thread para que o viewer não trave
                            t.detach();
                            break;
                        } 

                case '4':
                        {
                            if (!Led.getRunning()){
                                Led.setRunning(true);
                                std::cout<<" Led ativado"<<std::endl;
                                if (!pwm_BlinkLed.enable())
                                    std::cout<<" [Error]: pwm"<<std::endl;
                            }
                            else{
                                Led.setRunning(false);
                                std::cout<<" Led desativado"<<std::endl;
                                pwm_BlinkLed.disable();
                            }                                               
                            break;                        
                        }                      
                
                case '.':
                case '>': // Incremento do pwm
                        incrementaFreqLed(Led, pwm_BlinkLed, useLed_LT2PR);
                        break;    

                case ',':    
                case '<': // Decrementa pwm
                    {
                        if (!Led.getRunning()){
                            std::cout<<" Led está desativado"<<std::endl;
                        }
                        else{    
                            int passo= 2;
                            long dutyCicle= pwm_BlinkLed.getDutyCicle();

                            if (dutyCicle>= passo)
                                dutyCicle -= passo;
                            else
                                std::cout<< "DutyCicle= 0%)";

                            pwm_BlinkLed.setDutyCycle(dutyCicle);                            
                        }
                        break;                        
                    }                    

                case '+':
                case '=': // Incremento do pwm
                    {
                        int passo= 2;
                        long dutyCicle= pwm_PowerLed.getDutyCicle();

                        if (dutyCicle <= (100-passo))
                            dutyCicle += passo;
                        else
                            std::cout<< "DutyCicle= 100%)"; 

                        pwm_PowerLed.setDutyCycle(dutyCicle);                                 
                        break;
                    }

                case '-':
                case '_': // Decremento do pwm
                    {
                        int passo= 2;
                        long dutyCicle= pwm_PowerLed.getDutyCicle();

                        if (dutyCicle>= passo)
                            dutyCicle -= passo;
                        else
                            std::cout<< "DutyCicle= 0%)";

                        pwm_PowerLed.setDutyCycle(dutyCicle);                            
                        break;
                    }                    

                case 'l':
                case 'L':
                        limparTela();
                        hab_exibe_menu= true;
                        break;

                case 'q':
                case 'Q':
                        running = false;
                        std::cout << "Saindo do programa...." << std::endl;
                        break;

                default:
                        std::cout << "Comando invalido!" << std::endl;
                        break;
            }
        }
    }

    // *********** Finalização e limpeza:
    // Fecha a câmera:  
    camera.stop();

    // Para o gerador de frame:
    cd_frame_generator.stop();

    // Libera o GPIO da Jetson com segunraça. Este ´eum método da classe "configJetson":
    configuraGPIO_Jetson.liberaGPIO_Jetson(chip, gpios_actives);

    // Fecha todas as janelas:    
    cv::destroyAllWindows();

    // Desabilita o pwm
    pwm_BlinkLed.disable();

    return 0;
}