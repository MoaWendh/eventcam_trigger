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
void saveDataFileWithTrigger(EventCamera &cam, gpiod_line *line, const PARAMETROS_GERAIS &params) {
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
        std::string filename= "evecam_sn_" + serialNumber + "_" + time + ".raw";
        std::string full_path = nome_pasta + "/" + filename;
        

        // Inicia Gravação doarquivo de dados:
        if (cam.startRecording(full_path)){;
        
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
            if (cam.stopRecording()){
                std::cout << "Salvando dados ....." << std::endl;
                std::cout << "Dados salvos no arquivo: " << "\"" << filename << "\"" << std::endl;
            }
            else{
                std::cout << "!!!ERRO ao fechar arquivo:" << filename << std::endl;

            }
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


void incrementaPWM(PWM& pwm, const std::string funcao_PWM, bool use_led_potencia){
    if (use_led_potencia){
        int passo= 2;
        long dutyCicle= pwm.getDutyCicle();
        // Limita o duty-cicle em 10% caso esteja sendo usado o LED de potencia LT2PR:
        if (dutyCicle<10){
            if (dutyCicle <= (100-passo)){
                dutyCicle += passo;
                std::cout<< "Duty-Cicle "<< funcao_PWM << "= "<< dutyCicle << std::endl;
            }                     
            else
                std::cout<< "Duty-Cicle= 100%)" << std::endl;; 
            pwm.setDutyCycle(dutyCicle);
        }  
        else
            std::cout<< "[Led LT2PR] Duty-cicle atingiu o valor máximo de 10." <<std::endl;
    }    
    else{
        int passo= 2;
        long dutyCicle= pwm.getDutyCicle();

        if (dutyCicle <= (100-passo)){
            dutyCicle += passo;
            std::cout<< "Duty-Cicle= "<< funcao_PWM << "= "<< dutyCicle << std::endl; 
        }               
        else
            std::cout<< "Duty-Cicle= 100%)"; 
        pwm.setDutyCycle(dutyCicle);
    }

}


void decrementaPWM(PWM& pwm, const std::string funcao_PWM){ 
    int passo= 2;
    long dutyCicle= pwm.getDutyCicle();

    if (dutyCicle>= passo){
        dutyCicle -= passo;
        std::cout<< "Duty-Cicle "<< funcao_PWM << "= "<< dutyCicle << std::endl; 
    }            
    else
        std::cout<< "Duty-Cicle= 0%)";

    pwm.setDutyCycle(dutyCicle);                            

}

void configuraPWM(PWM &pwm, const std::string &path, long periodo, long duty_cicle, const std::string &canal, bool enable){
    // Primeiro define o chip de trabalho com o path referente a este periférico:
    pwm.setPathFileChip(path);

    // Inicializa o chip que controla o pwm:
    pwm.inicializa_canal();
    usleep(100000);

    // Ajusta o período do pwm A:
    if (pwm.setPeriodo(periodo))
        std::cout << "Periodo PWM "<< canal.c_str() << "= "<< periodo << std::endl;
    else
        std::cout << "[Erro] Não foi possível ajustar periodo pwm A." << std::endl;     
    usleep(100000);

    // Define o dutyciclo conforme o valor da variável:
    if (pwm.setDutyCycle(duty_cicle))
        std::cout << "Duty-Cicle PWM " << canal.c_str() << "= "<< duty_cicle << std::endl;
    else
        std::cout << "[Erro] Não foi possível ajustar o duty-cicle do pwm A." << std::endl;                
    usleep(100000);

    //if (enable)
      //  pwm.enable();

}


// Funcção principal:
int main(int argc, char *argv[]) {
    // Rotina que limpa o terminal:
    limparTela();

    PARAMETROS_GERAIS parametros_gerais;

    // Declaração variaveis booleans do tipo atomic:
    std::atomic<bool> useCamera_Conv(true);
    std::atomic<bool> useCamera_Event(true);

    // Flag que habilita ou não o menu, apenas para testes:
    bool hab_exibe_menu= true;  
    
    // Flaque que inicializa o loop while:
    bool running = true; 

    //********************************************************************************************/
    //********** Configuração da camera convencional FLIR (Model BlackFly BFS-U3-04S2M) **********/
    //********************************************************************************************/
    std::unique_ptr<ConvCamera> convCam_01 = nullptr;

    // Instancia o  objeto da camera convencional apenas se estiver habilitado este procedimento.
    if (useCamera_Conv.load()){
        // Cria a instância da classe "camera_conv" e atribuímos ao ponteiro do main:
        convCam_01 = std::make_unique<ConvCamera>(parametros_gerais.serialNumber_conv_cam_01);

        // Caham o método open definido na classe:
        if (!convCam_01->open()) {
            std::cerr << "[Error] Falha ao abrir a camera USB." << std::endl;
            convCam_01.reset(); // Destrói o objeto se a abertura falhar
        } else {
            convCam_01->exibir_configuracao();
        }
    }    



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

    // Instanciando os objetos PWM com a classe PWM:
    PWM pwm_BlinkLed;
    PWM pwm_PowerLed;

    // Chama função para configurar os canais PWMs: 
    // Configura canal PWM para trabalhar com o pulso do Led:
    configuraPWM(pwm_BlinkLed, parametros_gerais.channelToExport_A, parametros_gerais.periodo_PWM_A, parametros_gerais.dutyCicle_PWM_A, "Canal A", false);
    // Configura canal PWM para trabalhar com o controle da potência do Led:
    configuraPWM(pwm_PowerLed, parametros_gerais.channelToExport_B, parametros_gerais.periodo_PWM_B, parametros_gerais.dutyCicle_PWM_B, "Canal B", true);

   

    //********************************************************************************************/
    //*************************** Configuração da camera de eventos ******************************/
    //********************************************************************************************/

    // Instancia o objeto event_cam_xx para operar com a camera de eventos:
    EventCamera event_cam_01(parametros_gerais.serialNumber_event_cam3);
   
    // Booleano que define se a câmera usada é HD ou VGA:    
    bool sensorHD= false;    

    // 1- Instancia um objeto "camera" da classe "Metavision::camera":
    //Metavision::Camera camera_01;


    // Chama função para inicializar a camera de eventos:
    if (!event_cam_01.openEventCam()){
        std::cerr << "Não foi possível abrir camera de eventos." << std::endl;
    }

    // Captura alguns parametros, definidos em paramsEvCam_01, da camera de eventos :
    event_cam_01.getParametrosGeraisEventCam();
    

    // Carregando os biases na camera de eventos definidos em setings.json:
    if (!event_cam_01.setBias())
        std::cout << "ERRO!! Biases não gravado na câmera." << std::endl;
    
    Metavision::CDFrameGenerator cd_frame_generator(event_cam_01.getWidth(), event_cam_01.getHeight());

    
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
    event_cam_01.setCDCallback([&](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
        if (show_viewer) {
            cd_frame_generator.add_events(ev_begin, ev_end);
        }
    });


    // Habilita Trigger via HAL API
    if(event_cam_01.enableHardwareTrigger()){
          std::cout << std::endl;
        std::cout << "Trigger habilitado para ambas as bordas, Risinge e Falling edge." << std::endl;      
    }

    // Start camera de eventos:
    event_cam_01.start();

    // Dummy Trigger: "Acorda" o canal de trigger da Metavision
    gpiod_line_set_value(gpios_actives.triggerEventCam, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpiod_line_set_value(gpios_actives.triggerEventCam, 0);

    // Aguarda o sistema processar esse evento interno antes de liberar o menu
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


    std::cout << "Camera inicializada" << std::endl;

    // Instancia o objeto da classe LighCotroller: 
    LightController ledLight;

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
            showMenu(pin_TriggerEventCam, pin_PiscaLed, parametros_gerais.duracao_pulso_trigger);
            hab_exibe_menu= false;
        }

        // Máquina de estados do menu principal:
        if (key!=-1){
            char my_char= static_cast<char>(key);

            switch (my_char)
            {
                case '1':
                        // Efetua a leitura dos biases da camera de eventos:
                        event_cam_01.readCameraBiases();
                        break;
                
                case '2':
                        // Chama função para configurar, enviar, os biases à camera de eventos:                    
                        event_cam_01.setBias();
                        break;

                case '3':
                        {
                            std::cout << "\nTrigger Com duração de: "<< parametros_gerais.duracao_pulso_trigger << "us" << std::endl;

                            // Neste caso é melhor ativar uma thread usando uma função lambda, pois são passadas mais de uma função para ela:
                            std::thread t([&]() { 
                                for (int ctCiclo=0;  ctCiclo < parametros_gerais.numero_ciclos_trigger; ctCiclo++){                           
                                    // Primeira ativa a projeção da luz estruturada:
                                    ativaLedLight(ledLight, pwm_BlinkLed, pwm_PowerLed);                                     
                                    //Passando o objeto camera, o ponteiro da linha GPIO e a duração do pulso trigger
                                    saveDataFileWithTrigger(event_cam_01, gpios_actives.triggerEventCam, parametros_gerais);                                                               
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
                        incrementaPWM(pwm_PowerLed, "Tensão do Led", "false");                                 
                        break;

                case '-':
                case '_': // Decremento o pwm que controla a tensão analogica do Led (0 a 10V) ou laser (o a 5V):
                        decrementaPWM(pwm_PowerLed, "Tensão do Led");
                        break;                      

                case 'l':
                case 'L':
                        // Chama função para limpar o terminal: 
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
    event_cam_01.stop();

    // Para o gerador de frame:
    cd_frame_generator.stop();

    // Fecha todas as janelas:    
    cv::destroyAllWindows();

    // Desabilita o pwm
    pwm_BlinkLed.disable();

    return 0;
}