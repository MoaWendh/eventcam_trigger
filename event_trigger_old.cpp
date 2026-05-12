#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <vector>

#include "Spinnaker.h"
#include "conv_camera.h"
#include "event_camera.h"
#include "controlIO.h"
#include "parametros.h"


// Função que exibe o menu de opções:
void showMenu(int pinTrigger, int pinLed, int64_t duracao){     
    std::cout << ""<< std::endl;
    std::cout << "******************** Menu  ********************"<< std::endl;
    std::cout << " 1 - Ler biases da câmera" << std::endl;
    std::cout << " 2 - Gravar biases na câmera" << std::endl;   
    std::cout << " 3 - Trigger: iniciar gravacao de eventos (.raw)" << std::endl;       
    std::cout << " 4 - Start/Stop blink Led"<< std::endl;
    std::cout << " 5 - Captura imagem pela cam. convencinal"<< std::endl;    
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


// Simples rotina para limpar a tela:
void limparTela() {
    // \033[2J: Limpa a tela inteira
    // \033[H: Move o cursor para a posição inicial (canto superior esquerdo)
    std::cout << "\033[2J\033[H" << std::flush;
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



// 
void saveData_Stereo_TriggerHW(EventCamera &cam_01 , EventCamera &cam_02, gpiod_line *line, const PARAMETROS_GERAIS &params) {
    try {
        // Obter data para o nome da pasta e do arquivo:
        auto agora = std::chrono::system_clock::now();
        auto tempo_t = std::chrono::system_clock::to_time_t(agora);
        struct tm *info = std::localtime(&tempo_t);

        // gera o nome do diretório:
        std::stringstream ss_pasta;
        ss_pasta << "data_evecam_" << std::put_time(info, "%d_%m_%Y");
        std::string nome_pasta = ss_pasta.str();

        // Criar a pasta se ela não existir
        if (!std::filesystem::exists(nome_pasta)) {
            std::filesystem::create_directory(nome_pasta);
        }

        // Gera o nome do arquivo de dados .raw:
        std::string serialNumber_01= cam_01.getSerial();
        std::string serialNumber_02= cam_02.getSerial();
        std::stringstream ss_time;
        ss_time << std::put_time(info, "%H%M%S");
        std::string time= ss_time.str();
        std::string filename_01= "evecam_sn_" + serialNumber_01 + "_" + time + ".raw";
        std::string filename_02= "evecam_sn_" + serialNumber_02 + "_" + time + ".raw";
        //std::string full_path_01 = nome_pasta + "/01/" + filename_01;
        //std::string full_path_02 = nome_pasta + "/02/" + filename_02;
        std::string full_path_01 = nome_pasta + filename_01;
        std::string full_path_02 = nome_pasta + filename_02;

        bool ok_01 = cam_01.startRecording(full_path_01);   
        bool ok_02 = cam_02.startRecording(full_path_02);

        // Inicia Gravação doarquivo de dados:
        if (ok_01 && ok_02){
            std::cout << "Salvando dados ....." << std::endl;

            // Pré-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg para garantir que o arquivo foi aberto e o buffer inicializou:
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pre_trigger));
            

            // *******************Início do pulso de trigger**************** 
            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 1);  

            // Mantém o pulso em nivel alto pelo tempo especificado em "duracao_pulso_trigger_microSeg":
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pulso_trigger));

            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 0);
            // ********************Fim do pulso de trigger******************

            // Pós-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg antes de fecahr o arquivo de dados: 
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pos_trigger));

            // Finaliza a Gravação e fecha arquivo de dados:
            if (cam_01.stopRecording() &cam_02.stopRecording()){
                std::cout << "Dados salvos no arquivo: " << "\"" << full_path_01 << "\"" << std::endl;
                std::cout << "Dados salvos no arquivo: " << "\"" << full_path_02 << "\"" << std::endl;
            }
            else{
                std::cout << "!!!ERRO ao fechar arquivo:" << std::endl;
            }
        }

    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}


// 
void saveData_Mono_TriggerHW(EventCamera &cam, gpiod_line *line, const PARAMETROS_GERAIS &params) {
    try {
        // Obter data para o nome da pasta e do arquivo:
        auto agora = std::chrono::system_clock::now();
        auto tempo_t = std::chrono::system_clock::to_time_t(agora);
        struct tm *info = std::localtime(&tempo_t);

        // gera o nome do diretório:
        std::stringstream ss_pasta;
        ss_pasta << "data_evecam_" << std::put_time(info, "%d_%m_%Y");
        std::string nome_pasta = ss_pasta.str();

        // Criar a pasta se ela não existir
        if (!std::filesystem::exists(nome_pasta)) {
            std::filesystem::create_directory(nome_pasta);
        }

        // Gera o nome do arquivo de dados .raw:
        std::string serialNumber= cam.getSerial();
        std::stringstream ss_time;
        ss_time << std::put_time(info, "%H%M%S");
        std::string time= ss_time.str();
        std::string filename= "evecam_sn" + serialNumber + "_" + time + ".raw";
        std::string full_path= nome_pasta + filename;

        // Inicia Gravação doarquivo de dados:
        if (cam.startRecording(full_path)){
            std::cout << "Salvando dados ....." << std::endl;

            // Pré-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg para garantir que o arquivo foi aberto e o buffer inicializou:
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pre_trigger));
            

            // *******************Início do pulso de trigger**************** 
            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 1);  

            // Mantém o pulso em nivel alto pelo tempo especificado em "duracao_pulso_trigger_microSeg":
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pulso_trigger));

            // Transição do trigger para nível alto:
            gpiod_line_set_value(line, 0);
            // ********************Fim do pulso de trigger******************

            // Pós-trigger: aguarda um tempo em microsegundos definido em duracao_PosTrigger_microSeg antes de fecahr o arquivo de dados: 
            std::this_thread::sleep_for(std::chrono::microseconds(params.duracao_pos_trigger));

            // Finaliza a Gravação e fecha arquivo de dados:
            if (cam.stopRecording())
                std::cout << "Dados salvos no arquivo: " << "\"" << full_path << "\"" << std::endl;
            else
                std::cout << "!!!ERRO ao fechar arquivo:" << std::endl;
        }

    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}


bool ativaLedLight(LightController& led, PWM& pwm_A, PWM& pwm_B){
    if (!pwm_A.getStatus()){
        if (pwm_A.enable())           
            std::cout<<" PWM Blink: Ativado." <<std::endl;
        else{
            std::cout<<" [Error]: não foi possível ativar o PWM blink led"<< std::endl;
            led.setRunning(false);
            return false;
        }    
    }

    if (!pwm_B.getStatus()){
        if (pwm_B.enable())
            std::cout<<" PWM voltage: Ativado." <<std::endl;
        else{
            std::cout<<" [Error]: não foi possível ativar o PWM que controla a tensão do led." << std::endl;
            led.setRunning(false);
            return false;
        }    
    }

    led.setRunning(true);
    return true;
}



void desativaLedLight(LightController& led, PWM& pwm_A, PWM& pwm_B){
    if (pwm_A.getStatus()){
        pwm_A.disable();
        std::cout<<" PWM blink: Desativado."<< std::endl; 
    }
    else
        std::cout<<" PWM blink já está desativado." << std::endl;

    if (pwm_B.getStatus()){
        pwm_B.disable();
        std::cout<<" PWM voltage: Desativado."<< std::endl; 
    }
    else
        std::cout<<" PWM voltage já está desativado." << std::endl;        
    led.setRunning(false); 
}


// Incrementa o valor do PWM dado em percentual, o parâmetro "use_led_potencia" é usado para limitar o duty-cycle em 10% 
// caso esteja sendo usado o LED de potencia LT2PR, para evitar danos ao led:
void incrementaPWM(PWM& pwm, const std::string funcao_PWM, bool use_led_potencia){
    if (use_led_potencia){
        int passo= 1;
        long dutyCycle= pwm.getDutyCycle();
        // Limita o duty-cycle em 10% caso esteja sendo usado o LED de potencia LT2PR:
        if (dutyCycle<=(10-passo)){
            dutyCycle += passo;                    
            pwm.setDutyCycle(dutyCycle);
        }  
        else
            std::cout<< "[Led LT2PR] Duty-cycle atingiu o valor máximo de 10." <<std::endl;
    }    
    else{
        int passo= 1;
        long dutyCycle= pwm.getDutyCycle();

        if (dutyCycle <= (100-passo)){
            dutyCycle += passo;
            pwm.setDutyCycle(dutyCycle);
        }               
        else
            std::cout<< "Duty-Cycle= 100%)"; 
    }

}


// Decrementa o valor do PWM dado em percentual:
void decrementaPWM(PWM& pwm, const std::string funcao_PWM){ 
    int passo= 2;
    long dutyCycle= pwm.getDutyCycle();

    if (dutyCycle>= passo){
        dutyCycle -= passo;
        pwm.setDutyCycle(dutyCycle);
    } 
    else if (dutyCycle==1){
        dutyCycle= 0;
        pwm.setDutyCycle(dutyCycle);
    }
    else
        std::cout<< "Duty-Cycle= 0%)";                               

}


// Verificação se os pinos forma configurados ok, por segurança:
int confirma_gpios_actives(GPIO_Lines &gpios_actives, gpiod_chip *chip){
    if (gpios_actives.triggerEventCam == nullptr || gpios_actives.piscaLed == nullptr || gpios_actives.triggerNormalCam == nullptr || 
        gpios_actives.controlMotor01 == nullptr  || gpios_actives.controlMotor02 == nullptr ) {
            std::cerr << "[ERRO] Falha crítica na inicialização dos GPIOs. Abortando!!!!" << std::endl;
        if (chip)
            gpiod_chip_close(chip);
        return 0;
    }
    return 1;
}

// Esa fuanção é executada antes de tentar instanciar uma camera de eventos, para verificar se há câmeras 
// conectadas no barramento USB, evitando erros de conexão ou falhas de hardware.
// Caso não sejam detectadas cameras, o progrma é abortado.
bool detectaCamerasConectadas(){
    try{
        // Tenta capturar a lista de seriais de todas as câmeras conectadas:
        Metavision::DeviceDiscovery::SerialList dispositivos= Metavision::DeviceDiscovery::list();
        if (!dispositivos.empty()){
            int numDevices= dispositivos.size();
            std::cout <<  std::endl;            
            std::cout << "Cameras de Eventos Conectadas: "<< numDevices << std::endl;
            for (int ct=0; ct<numDevices; ct++){
                auto it = std::next(dispositivos.begin(), ct); 
                // Exibe apenas os 8 primeiros caracteres do número de série, que são os mais relevantes para identificação. O restante é sufixo e pode variar entre modelos ou versões.
                std::cout << " - " << it->substr(0, 11) << " | Nº Série: " << it->substr(32, 40) << std::endl;
            }
            std::cout << std::endl; 
            return true;
        }
        else{
            std::cerr << "Nenhuma câmera de eventos detectada no barramento USB." << std::endl; 
            return false;
        }
    }
    catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha ao escanear barramento USB." << e.what() << std::endl;
        return false;
    }    
}


// Este menu é exibido no terminal do OpenCV junamente com os frames capturados pela câmera convencional, 
// para facilitar a visualização e controle das opções de trigger e configuração do LED. 
// Ele é desenhado diretamente sobre o frame da câmera convencional, utilizando as funções de desenho do OpenCV, 
// como putText e circle. O menu inclui as opções de controle, bem como um indicador "LIVE" que pisca para mostrar que a captura está ativa. 
// A função MenuFrame é chamada a cada frame capturado pela câmera convencional para atualizar o menu em tempo real.
void MenuFrame(const cv::Mat& input, cv::Mat& output) {
    if (input.empty()) return;

    int largura_menu = 300;

    // 1. Garante que o input seja convertido para 3 canais (Colorido) 
    // para podermos desenhar em Verde/Vermelho.
    cv::Mat input_color;
    if (input.channels() == 1) {
        cv::cvtColor(input, input_color, cv::COLOR_GRAY2BGR);
    } else {
        input_color = input;
    }

    // 2. Cria a moldura com a borda preta à direita
    cv::copyMakeBorder(input_color, output, 0, 0, 0, largura_menu, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // 3. Configurações de texto
    int x_start = input_color.cols + 20;
    int y_start = 40;
    int espacamento = 30;

    std::vector<std::string> itens = {
        "*** MENU DE CONTROLE ***",
        "1 - Ler biases",
        "2 - Gravar biases",
        "3 - Trigger (Gravacao)",
        "4 - Start/Stop Blink LED",
        "5 - Captura Convencional",
        "-----------------------",
        "+ / - : Potencia LED",
        "> / < : Duracao Pulso",
        "L : Limpa Tela",
        "Q : Sair"
    };

    // 4. Desenha o texto
    for (size_t i = 0; i < itens.size(); ++i) {
        cv::putText(output, 
                    itens[i], 
                    cv::Point(x_start, y_start + (int)(i * espacamento)), 
                    cv::FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    cv::Scalar(0, 255, 0), // Verde
                    1, 
                    cv::LINE_AA);
    }

    // 5. Indicador "LIVE" (Círculo vermelho piscando no canto superior direito)
    // Usamos o tempo do sistema para fazer piscar
    auto milis = std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    
    if ((milis / 500) % 2 == 0) { // Pisca a cada 500ms
        cv::circle(output, cv::Point(output.cols - 20, 20), 7, cv::Scalar(0, 0, 255), -1);
        cv::putText(output, "LIVE", cv::Point(output.cols - 60, 25), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    }
}




// Funcção principal:
int main(int argc, char *argv[]) {
    // Rotina que limpa o terminal:
    limparTela();

    // Carrgea os parametros gerais definidos na struct "PARAMETROS_GERAIS" do header "parametros.h":
    PARAMETROS_GERAIS parametros_gerais;


    //********************************************************************************************/
    //****************************** Configuração GPIO da Jetson *******************************/
    //********************************************************************************************/

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

    // Por segurança, verifica se as linhas "gpios_actives" foram ativadas, configuradas ok, caso contrario o programa é abortado para evitar falhas de hardware:
    if (!confirma_gpios_actives(gpios_actives, chip))
        return 1; 


    // ***********************************************************************************************/
    // ******************************** Configuraçaõ do PWM ******************************************/
    // ***********************************************************************************************/    
    // Configuração do PWM direta via Sysfs (Linux Kernel), que utiliza a interface de arquivos virtuais do 
    // sistema operacional para manipular registradores de hardware nativos. Este método elimina 
    // dependências externas, garantindo que o sinal de PWM seja gerado de forma estável por hardware 
    // independente, sem sobrecarga da CPU.

    // Define se está usando o led de potencia LT2PR da Opto Engineering
    bool useLed_LT2PR= true;

    // Instanciando os objetos PWM com a classe PWM:
    PWM pwm_BlinkLed(parametros_gerais.periodo_PWM_A, parametros_gerais.dutyCycle_PWM_A, parametros_gerais.channelToExport_A, "Canal A");
    PWM pwm_PowerLed(parametros_gerais.periodo_PWM_B, parametros_gerais.dutyCycle_PWM_B, parametros_gerais.channelToExport_B, "Canal B");



    //********************************************************************************************/
    //********** Configuração da camera convencional FLIR (Model BlackFly BFS-U3-04S2M) **********/
    //********************************************************************************************/
    // Instancia um objeto do tipo COnvCamera: 
    ConvCamera* convCam_01 = nullptr;

    // Instanciar o Singleton: é uma instância única do sistema que gerencia a comunicação com o hardware para acessar a camera.
    // Carrega a Camada de Transporte: Inicializa os drivers e protocolos (USB3, GigE) necessários para detectar e conversar com as câmeras.
    // Ponto de Entrada: É o objeto obrigatório para listar dispositivos e gerenciar o ciclo de vida da SDK.
    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();

    // Varredura das portas: Escaneia as interfaces (USB/GigE) em busca de câmeras conectadas.
    // Cria uma lista contendo referências (objetos CameraPtr) para todos os dispositivos encontrados.
    // Permite localizar uma câmera específica, por exemplo pelo nº de Serie para começar a operá-la.
    Spinnaker::CameraList camList = system->GetCameras();

    // Instancia o  objeto da camera convencional apenas se estiver habilitado este procedimento.
    if (parametros_gerais.useCamera_Conv){

        // Busca Seletiva: Percorre a lista de câmeras detectadas procurando o identificador único, Serial Number, definido nos parametros_gerais.
        // Verifica se a câmera desejada está conectada e disponível antes de iniciar a operação.
        Spinnaker::CameraPtr pCamBase = camList.GetBySerial(parametros_gerais.serialNumber_conv_cam_01);

        // Testa se a camera com o referido nº de séri foi encontrada:
        if (!pCamBase.IsValid()) {
            std::cerr << "Câmera não encontrada!" << std::endl;
            return -1;
        }

        // Conversão de Tipo: Transforma o ponteiro genérico da SDK, Spinnaker::Camera*, no tipo definido pela minha classe ConvCamera: 
        // Permite acesso aos métodos e atributos personalizados especificos da classe ConvCamera e também da classe original da FLIR, que foi herdada.
        // Desta forma, a classe ConmvCamera terá acesso a toda as funções e metodos da classe Spinnaker::Camera:
        convCam_01 = static_cast<ConvCamera*>(pCamBase.get());
        
        // Se ok, será exibida a configuraçã da camera convencional 
        if (convCam_01){
            convCam_01->Init();
            std::cout<< std::endl;
            std::cout<< "*** Câmera convencional: ***"<< std::endl;
            convCam_01->exibir_modelo_camera();
            std::cout<< std::endl;
            convCam_01->DeInit();
        }
    }    



    //********************************************************************************************/
    //*************************** Configuração da camera de eventos ******************************/
    //********************************************************************************************/

    // Se não detectar camera de eventos no barramente USB, o programa é abortado para evitar falhas de hardware:
    if (!detectaCamerasConectadas()){
        return 1;
    }

    // Instancia o objeto event_cam_xx para operar com a camera de eventos:
    // Esta "Classe EventCamera" foi criada para tratar dos objeos, atributops e métodos do SDK Metavision: 
    // O construtor recebe um parametro booleano do tipo "argumento padrão", só é passado quando for necessário,
    // Ou seja, quando a câmera é master, para configurar o sincronismo de hardware. 
    // Caso contrário, o valor padrão é "false", ou seja, a câmera é slave.
    // Define se o sistema sera mono ou estéreo:
    const int numCams= 2;

    // Apenas detecta as câmeras que estão conectadas ao barramento USB:
    EventCamera event_cam[numCams] = {
                                    EventCamera(parametros_gerais.serialNumber_event_cam3, "Left", true),
                                    EventCamera(parametros_gerais.serialNumber_event_cam2, "Right")
    };


    for (int i=0; i<numCams; i++){
        // COnfigura, carrega, os biases na camera de eventos definidos em setings.json:
        if (!event_cam[i].setBias(parametros_gerais)){
            std::cerr << "Câmera de eventos[ " << i << "] não encontrada!" << std::endl;
            return -1;
        }

        // Imporante!!! Esta função habilita o Trigger via HAL da API:
        event_cam[i].enableHardwareTrigger();

        // Esta função configura o sincronismo de tempo entre as cameras de eventos via HAL da API: 
        // onde a câmera master é a "Left" e a slave é a "Right":
        event_cam[i].configSincronismo();
    }

    std::mutex cd_frame_mutex;
    cv::Mat cd_frame;
    std::atomic<bool> show_viewer{true}; 
    std::string window_name = "Visualizacao em Tempo Real";

    // Instancia o objeto cd_frame_generator da classe Metavision::CDFrameGenerator, que é um gerador de frames interno da SDK da Metavision.
    // Ele acumula os eventos CD e gera frames para visualização em tempo real.    
    Metavision::CDFrameGenerator cd_frame_generator(event_cam[0].getWidth(), event_cam[0].getHeight());
    
    // Define o tempo de exposição dos eventos acumulados.
    // Aguarda acumular 10.000 us (10ms) de eventos para exibição: 
    cd_frame_generator.set_display_accumulation_time_us(10000);

    // Gerador de frames interno
    cd_frame_generator.start(30, [&](const Metavision::timestamp &ts, const cv::Mat &frame) {
        if (show_viewer) {
            std::unique_lock<std::mutex> lock(cd_frame_mutex);
            frame.copyTo(cd_frame);
        }
    });

    
    // Callback para processar eventos CD
    event_cam[0].setCDCallback([&](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
        if (show_viewer) {
            cd_frame_generator.add_events(ev_begin, ev_end);
        }
    });


    // Start as camera de eventos. A partir deste momento, as câmeras começam a captar os eventos e o gerador de frames 
    // interno da SDK da Metavision começa a processar os eventos para exibição em tempo real:
    for (int i=0; i<numCams; i++){
        event_cam[i].callStart();
    }         

    // Sequencia Dummy Trigger para "acordar" o canal de trigger da Metavision:
    gpiod_line_set_value(gpios_actives.triggerEventCam, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpiod_line_set_value(gpios_actives.triggerEventCam, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Instancia o objeto da classe LighCotroller: 
    LightController ledLight;



    
    // 1. Dimensões baseadas no sensor (Assumindo que ambas as cams têm a mesma resolução)
    int cam_w = event_cam[0].getWidth();
    int cam_h = event_cam[0].getHeight();
    int largura_menu = 300;

    // 2. Cria o Dashboard Único (Lado a Lado: Cam_Esquerda | Cam_Direita | Menu)
    // Se estiver em modo mono, a parte da direita simplesmente ficará preta ou com aviso.
    cv::Mat dashboard = cv::Mat::zeros(cam_h, (cam_w * 2) + largura_menu, CV_8UC3);

    // 3. Define as ROIs (Sub-regiões do Dashboard)
    // Elas não ocupam memória nova, são apenas "janelas" para dentro da matriz dashboard.
    cv::Mat roi_L    = dashboard(cv::Rect(0, 0, cam_w, cam_h));               // Área da Cam 0 (Master)
    cv::Mat roi_R    = dashboard(cv::Rect(cam_w, 0, cam_w, cam_h));           // Área da Cam 1 (Slave)
    cv::Mat roi_menu = dashboard(cv::Rect(cam_w * 2, 0, largura_menu, cam_h)); // Área do Menu

    // 4. Desenha o Menu Estático na roi_menu APENAS UMA VEZ
    std::vector<std::string> itens_menu = {
        "*** SISTEMA METRA-EVENT ***",
        "1 - Ler Biases",
        "2 - Gravar Biases",
        "3 - Trigger REC",
        "4 - Blink LED",
        "5 - Cam. Convencional",
        "-----------------------",
        "+ / - : Potencia LED",
        "> / < : Duracao Pulso",
        "Q - Sair"
    };

    for (size_t i = 0; i < itens_menu.size(); ++i) {
        cv::putText(roi_menu, itens_menu[i], cv::Point(20, 40 + i * 35), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }

    // Única janela necessária
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);


    //********************************************************************************************  
    //****************************** Loop principal **********************************************
    //********************************************************************************************/
    // Flag que habilita a execução do loop while:
    bool running = true; 
    
    // Loop principal:    
    while(running) {

        int key = cv::waitKey(1);

        if (show_viewer) {
            std::unique_lock<std::mutex> lock(cd_frame_mutex);
            
            // Atualiza Câmera Esquerda (Master)
            if (!cd_frame.empty()) {
                if (cd_frame.channels() == 1)
                    cv::cvtColor(cd_frame, roi_L, cv::COLOR_GRAY2BGR);
                else
                    cd_frame.copyTo(roi_L);
            }

            // Se estiver em modo Mono, coloque um aviso na segunda metade
            if (numCams < 2) {
                cv::putText(roi_R, "SISTEMA MONO", cv::Point(cam_w/4, cam_h/2), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            } 
            else {
                // Se tiver a segunda cam, você usaria o cd_frame_02 aqui:
                // cd_frame_02.copyTo(roi_R);
            }
        }

        // Exibe o Dashboard COMPLETO em uma única janela
        cv::imshow(window_name, dashboard);           


        /// Exibe menu de escolha:
        if (parametros_gerais.hab_exibe_menu){
            //limparTela();
            showMenu(pin_TriggerEventCam, pin_PiscaLed, parametros_gerais.duracao_pulso_trigger);
            parametros_gerais.hab_exibe_menu= false;
        }

        // Máquina de estados do menu principal:
        if (key!=-1){
            char my_char= static_cast<char>(key);

            switch (my_char)
            {
                case '1':
                        // Efetua a leitura dos biases da camera de eventos:
                        event_cam[0].readCameraBiases();
                        break;
                
                case '2':
                        // Chama função para configurar, enviar, os biases à camera de eventos:                    
                        event_cam[0].setBias(parametros_gerais);
                        break;

                case '3':
                        {
                            std::cout << "\nTrigger Com duração de: "<< parametros_gerais.duracao_pulso_trigger << "us" << std::endl;

                            // Usa-se uma thread com função lambda, onde:
                            // [&] = Captura: ela captura todas as variáveis e métodos, por referencia, no escopo de main(), pois ela utilzia várias funções e variáveis;
                            // () = Parâmetros: Sem nenhum parâmetro como argumento;
                            // {} = Corpo: onde são chamadas as funções.:
                            std::thread t([&]() { 
                                for (int ctCiclo=0;  ctCiclo < parametros_gerais.numero_ciclos_trigger; ctCiclo++){                           
                                    // Primeira ativa a projeção da luz estruturada:
                                    ativaLedLight(ledLight, pwm_BlinkLed, pwm_PowerLed);
                                    // Chama a gravação do dados da câmera de eventos, que é feita em paralelo, ou seja, enquanto a câmera de eventos está gravando os 
                                    // dados no arquivo .raw, o programa continua executando as próximas linhas de código, sem esperar a finalização da gravação.                                   
                                    saveData_Mono_TriggerHW(event_cam[0], gpios_actives.triggerEventCam, parametros_gerais); 
                                    // Por ultimo, desativa o projeção de luz estruturada:
                                    desativaLedLight(ledLight, pwm_BlinkLed, pwm_PowerLed);
                                }
                            });

                            // Desacoplar a thread para que o viewer não trave
                            t.detach();
                            break;
                        } 

                case '4':
                        {
                            if (!ledLight.getRunning())
                                ativaLedLight(ledLight, pwm_BlinkLed, pwm_PowerLed);
                            else
                                desativaLedLight(ledLight, pwm_BlinkLed, pwm_PowerLed);                                             
                            break;                        
                        }  
                        
                case '5':
                        {
                            // Captura uma imagem pela câmera convencional:
                            if (convCam_01){
                                convCam_01->Init();
                                convCam_01->capturarImagem();
                                convCam_01->DeInit();
                            }
                            else
                                std::cout << "Câmera convencional não instanciada." << std::endl;
                            break;                        
                        }        
                
                case '.':
                case '>': // Incrementa o pwm que controla o tempo de atuação do Led, duração do blink do Led
                        incrementaPWM(pwm_BlinkLed, "Blink Led", parametros_gerais.useLed_LT2PR);
                        break;    

                case ',':    
                case '<': // Decrementa o pwm que controla o tempo de atuação do Led, duração do blink do Led
                        decrementaPWM(pwm_BlinkLed, "Blink Led");
                        break;                    

                case '+':
                case '=': // Incrementa o pwm que controla a tensão analogica do Led (0 a 10V) ou laser (o a 5V):
                        incrementaPWM(pwm_PowerLed, "Tensão do Led", false);                                 
                        break;

                case '-':
                case '_': // Decremento o pwm que controla a tensão analogica do Led (0 a 10V) ou laser (o a 5V):
                        decrementaPWM(pwm_PowerLed, "Tensão do Led");
                        break;                      

                case 'l':
                case 'L':
                        // Chama função para limpar o terminal: 
                        limparTela();
                        parametros_gerais.hab_exibe_menu= true;
                        break;

                case 'q':
                case 'Q':
                        running = false;
                        std::cout <<std::endl;
                        break;

                default:
                        std::cout << "Comando invalido!" << std::endl;
                        break;
            }
        }
    }

    // Fecha a câmera:  
    for (int i=0; i<numCams; i++){
        event_cam[i].stop();
    }     


    // Para o gerador de frame:
    cd_frame_generator.stop();

    // Fecha todas as janelas:    
    cv::destroyAllWindows();

    // Desabilita o pwm
    pwm_BlinkLed.disable();

    return 0;
}