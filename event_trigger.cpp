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


// 
void saveDataFileWithTrigger(Metavision::Camera &cam, gpiod_line *line, int duracao_trigger_us, std::string serial) {
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
        
        // Pré-trigger: 50ms para garantir que o arquivo foi aberto e o buffer inicializou:
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        // Início do pulso de trigger, transição para nível alto:
        gpiod_line_set_value(line, 1);
        
        // Duração do Trigger em niivel alto:
        std::this_thread::sleep_for(std::chrono::milliseconds(duracao_trigger_us));

        // O trigger vai opara nivel baixo indicando final do trigger de eventos:
        gpiod_line_set_value(line, 0);

        // Pós-trigger: 0,1ms, aguarda 100us antes de fecahr o arquivo de dados: 
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        // Finaliza a Gravação e fecha arquivo de dados:
        cam.stop_recording();
        
        std::cout << "Salvo arquivo: " << "\"" << filename << "\"" << std::endl;
        
    } catch (const std::exception &e) {
        std::cerr << "[ERRO] Falha na thread de captura: " << e.what() << std::endl;
    }
}


// Funação que gera o menu de opções:
void showMenu(int pin, int64_t duracao){     
    std::cout << ""<< std::endl;
    std::cout << "******************** Menu  ********************"<< std::endl;
    std::cout << " H - Ativar o pino "<<  pin << " da Jetson (3,3V)"<< std::endl;        
    std::cout << " L - Desativar o pino "<<  pin << " da Jetson (0,0V)"<< std::endl;
    std::cout << " S - Iniciar gravacao de eventos com trigger (.raw)" << std::endl;
    std::cout << " Q - Sair do programa "<< std::endl;
    std::cout << "     Digite a opção: ";
};


// Funcção principal:
int main(int argc, char *argv[]) {
    limparTela();

    //Offsets para acesar os IOs:
    std::string chipIO= "gpiochip0";

    std::string serial_cam_1= "unknow";

    int numTrainOfPulses= 1;
    char myChoice;
    bool running = true;
    int64_t duracaoPulsoTrigger= 10;

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

    // Instancia um objeto "camera" da classe "Metavision::camera":
    Metavision::Camera camera;
    try {
        camera = Metavision::Camera::from_first_available();
        std::cout << "Camera inicializada." << std::endl;
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
        std::cout << "Trigger habilitado" << std::endl;
    }

    // Start camera de eventos:
    camera.start();
    std::cout << "Camera inicializada" << std::endl;



    //*********** Inicia configuração do barramenteo IO da Jetson: 
    // 1º. Abre o chip definido na string chipIO:
    struct gpiod_chip *chip = gpiod_chip_open_by_name(chipIO.c_str());
    if (!chip) {
        perror("Erro ao abrir gpiochip0!!!");
        return 1;
    }

    // 2º. Captura o pino, line, correspondente. Definido em active_line
    struct gpiod_line *line = gpiod_chip_get_line(chip, active_line);
    if (!line) {
        std::cerr << "Erro: Nao foi possivel obter a linha: " << active_line << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }

    // 3º. Faz um request da linha como SAÍDA:
    if (gpiod_line_request_output(line, "sync_trigger", 0) < 0) {
        perror("Erro ao configurar saída");
        return 1;
    }

    std::cout << "Jetson: trigger no pino: " << active_pin << std::endl;
    std::cout << "Jetson: trigger no SOC line: "<< active_line << std::endl;


    // Loop principal:
    while(running) {

        // Se o viewer estiver ativo, atualiza a janela sem travar o menu
        if (show_viewer) {
            std::unique_lock<std::mutex> lock(cd_frame_mutex);
            if (!cd_frame.empty()) {
                cv::imshow(window_name, cd_frame);
            }
            cv::waitKey(1); 
        }

        /// Exibe menu de escolha:
        if (hab_exibe_menu){
            //limparTela();
            showMenu(active_pin, duracaoPulsoTrigger);
            hab_exibe_menu= false;
        }

        // 2. Captura de Teclado via OpenCV (33ms de espera = ~30 FPS)
        // Isso substitui o std::cin e não trava o programa
        int key = cv::waitKey(100);

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
                /*    
                case 'v':
                case 'V':
                    show_viewer = true;
                    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
                    // Adicione esta linha para forçar a janela para o topo e atrair o foco:
                    cv::setWindowProperty(window_name, cv::WND_PROP_TOPMOST, 1);
                    cv::resizeWindow(window_name, geometry.get_width(), geometry.get_height());
                    std::cout << " Viewer aberto." << std::endl;
                    break;
                */
               /*    
                case 'f':        
                case 'F':
                    show_viewer = false;
                    cv::destroyWindow(window_name);
                    std::cout << " Viewer fechado." << std::endl;
                    hab_exibe_menu= true;
                    break;                
                */  

                case 's':
                case 'S':
                        {
                            std::cout << "\nSalvando dados .raw com trigger de: "<< duracaoPulsoTrigger << "ms" << std::endl;
            
                            // Passando o objeto camera, o ponteiro da linha GPIO e a duração do pulso trigger
                            std::thread t(saveDataFileWithTrigger, std::ref(camera), line, duracaoPulsoTrigger, serial_cam_1);
                            
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

    // Finalização e limpeza
    camera.stop();
    cd_frame_generator.stop();
    gpiod_line_release(line);
    gpiod_chip_close(chip);
    cv::destroyAllWindows();

    return 0;
}