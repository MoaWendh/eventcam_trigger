#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>

// Inclusões do Metavision e OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/hal/facilities/i_trigger_in.h>


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


// Funação que gera o menu de opções:
void showMenu(int pin, int64_t duracao){     
    std::cout << ""<< std::endl;
    std::cout << "******************** Menu  ********************"<< std::endl;
    std::cout << " H - Ativar o pino "<<  pin << " da Jetson (3,3V)"<< std::endl;        
    std::cout << " L - Desativar o pino "<<  pin << " da Jetson (0,0V)"<< std::endl;
    std::cout << " T - Iniciar gravacao de eventos com trigger (.raw)" << std::endl;
    std::cout << " Q - Sair do programa "<< std::endl;
    std::cout << "     Digite a opção: ";
};


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
            std::cout << "Salvo arquivo: " << "\"" << filename << "\"" << std::endl;
        }
        else{
            std::cout << "!!!ERRO ao fechar arquivo:" << filename << std::endl;

        }

    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}



// Função quew efetua a cofniguração do barramento GPIO da Jetson, ela retorna um ponteiro tipo gpio_line usado para acessar os pinos de IO:
struct gpiod_line* inicializaGPIO_Jetson(std::string chipIO, int active_line, int active_pin, struct gpiod_chip **chip_ptr){

    //********************* Inicia configuração do barramenteo IO da Jetson: 
    // 1º. Abre o chip definido na string chipIO e armazena no ponteiro fornecido pelo main:
    *chip_ptr = gpiod_chip_open_by_name(chipIO.c_str());

    // struct gpiod_chip *chip = gpiod_chip_open_by_name(chipIO.c_str());
    if (!(*chip_ptr)) {
        perror("Erro ao abrir gpiochip0!!!");
        return nullptr;
    }

    // 2º. Captura o pino, line, correspondente. Definido em active_line
    struct gpiod_line *line = gpiod_chip_get_line(*chip_ptr, active_line);
    if (!line) {
        std::cerr << "Erro: Nao foi possivel obter a linha: " << active_line << std::endl;
        gpiod_chip_close(*chip_ptr);
        return nullptr;
    }

    // 3º. Faz um request da linha como SAÍDA:
    if (gpiod_line_request_output(line, "sync_trigger", 0) == 0){
        //Garantir que o pino do IO da Jetson inicie em nivel baixo, pois ele pode iniciar com um valor qualquer:
        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: trigger no pino: " << active_pin << std::endl;
        std::cout << "Jetson: Estado inical do pino " << active_pin << "= 0V" << std::endl;
        std::cout << "Jetson: trigger no SOC line: "<< active_line << std::endl;        
    } 
    else {
        perror("Erro ao configurar saída");
        return nullptr;
    }

    return line;
}


// Funcção principal:
int main(int argc, char *argv[]) {
    limparTela();

    // Declara o ponteiro do chip aqui para poder ser fechado depois dentro do main:
    struct gpiod_chip *chip = nullptr;

    //Offsets para acesar os IOs:
    std::string chipIO= "gpiochip0";

    std::string serial_cam_1= "unknow";

    int numTrainOfPulses= 1;
    char myChoice;
    bool running = true;
    int64_t duracao_PulsoTrigger_microSeg= 30000; // em micro segundos
    int64_t duracao_PreTrigger_microSeg= 50000; // em micro segundos
    int64_t duracao_PosTrigger_microSeg= 50000; // em micro segundos

    bool hab_exibe_menu= true;

    //Seleção de pinos de IOs disponíveis
    int lineA= 43;  // GPIO13 PH.00 - Pino IO no Header= 33
    int lineB= 85;  // GPIO12 PN.01 - Pino IO no Header= 15
    int lineC= 105; // GPIO01 PQ.05 - Pino IO no Header= 29
    int lineD= 144; // GPIO01 PAC.06 - Pino IO no Header= 7

    int header_pinA= 33;
    int header_pinB= 15;
    int header_pinC= 29;
    int header_pinD= 7;

    //Define-se o pino, line, que será aivado:
    int active_line= lineD; //
    int active_pin= header_pinD;


    // Chama função "inicializaGPIO_Jetson" para configuração do barramento GPIO da Jetson:
    struct gpiod_line *line= inicializaGPIO_Jetson(chipIO.c_str(), active_line, active_pin, &chip);
    if (line == nullptr) {
        std::cerr << "Falha crítica na inicialização do GPIO. Abortando!!!!" << std::endl;
        return 1;
    }

    //Metavision::DeviceConfig config;
    //config.set("trigger_in_mode", "both");

    // Instancia um objeto "camera" da classe "Metavision::camera":
    Metavision::Camera camera;
    try {
        camera = Metavision::Camera::from_first_available();
        //std::cout << "Instanciado o objeto camera." << std::endl;
    } catch (const Metavision::CameraException &e) {
        std::cerr << "Erro ao abrir a camera: " << e.what() << std::endl;
        return 1;
    }

    // Capurar o numero de seria da camera instanciada:
    try {
        serial_cam_1 = camera.get_camera_configuration().serial_number;
        std::cout << "Nº Serial cam: " << serial_cam_1 << std::endl;
    } catch (...) {
        std::cout << "Nao foi possivel obter o serial via CameraConfiguration." << std::endl;
    }

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
        std::cout << "Trigger habilitado para ambas as bordas, Risinge e Falling edge." << std::endl;
    }

    /*
    Metavision::DeviceConfig config;
    config.set("trigger_in_mode", "both"); // Nome do parâmetro pode variar conforme o plugin
    camera = Metavision::Camera::from_first_available(config);
    */

    // Start camera de eventos:
    camera.start();
    std::cout << "Camera inicializada" << std::endl;

   
    // Loop principal:
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
            showMenu(active_pin, duracao_PulsoTrigger_microSeg);
            hab_exibe_menu= false;
        }

        // 2. Captura de Teclado via OpenCV (33ms de espera = ~30 FPS)
        // Isso substitui o std::cin e não trava o programa
        //int key = cv::waitKey(100);

        if (key!=-1){
            char my_char= static_cast<char>(key);

            switch (my_char)
            {
                case 'h':
                case 'H':
                    gpiod_line_set_value(line, 1);
                    std::cout << " Pino " << active_pin << " em nivel ALTO" << std::endl;
                    break;

                case 'l':
                case 'L':
                    gpiod_line_set_value(line, 0);
                    std::cout << " Pino " << active_pin << " em nivel BAIXO" << std::endl;
                    break;

                case 't':
                case 'T':
                        {
                            std::cout << "\nSalvando dados .raw com trigger de: "<< duracao_PulsoTrigger_microSeg << "ms" << std::endl;
            
                            // Passando o objeto camera, o ponteiro da linha GPIO e a duração do pulso trigger
                            std::thread t(saveDataFileWithTrigger, std::ref(camera), line, duracao_PulsoTrigger_microSeg, duracao_PreTrigger_microSeg, duracao_PosTrigger_microSeg, serial_cam_1);
                            
                            // Desacoplar a thread para que o viewer não trave
                            t.detach();
                            break;
                        } 
                        
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

    // Fecha o pino da Jetson
    if (line) 
        gpiod_line_release(line);
    
    // LIbera o chip reservado na Jetson:    
    if (chip)
        gpiod_chip_close(chip);

    // Fecha todas as janelas:    
    cv::destroyAllWindows();

    return 0;
}