#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp> // Biblioteca para JSON

// Inclusões do Metavision e OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_ll_biases.h>

using json = nlohmann::json;


// Struct para as linas, pinos, da Jetson:
struct GPIO_Lines {
    struct gpiod_line *trigger;
    struct gpiod_line *led;
};



// ESta classe LedContreoller serve apenas para piscar o led pelo barremano de IO da Jetso:
class LEDController {
private:
    std::atomic<bool> is_running{false};
    std::thread blink_thread;
    struct gpiod_line *line; // Guarda o pino do barramento da Jetson 
    int pulse_duration_ms; // Armazena a duração internamente

    // Função interna que roda na thread
    void run_blink() {
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

public:
    // Construtor configurando pino e tempo:
    LEDController(struct gpiod_line *gpio_line, int duration_ms) 
        : line(gpio_line), pulse_duration_ms(duration_ms) {}

    // Desturtor do objeto:
    ~LEDController() {
        stop();
    }

    void start() {
        if (is_running) {
            std::cout << "Pulso Led ativado.\n";
            return;
        }
        is_running = true;
        blink_thread = std::thread(&LEDController::run_blink, this);
    }

    void stop() {
        if (is_running) {
            is_running = false;
            if (blink_thread.joinable()) {
                blink_thread.join();
            }
        } else {
            std::cout << "Nenhuma thread ativa para parar!!!!\n";
        }
    }

    bool isActive() const {
        return is_running;
    }
};




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
    std::cout << " 3 - Ativar o pino "<<  pinTrigger << " da Jetson (3,3V)"<< std::endl;        
    std::cout << " 4 - Desativar o pino "<<  pinTrigger << " da Jetson (0,0V)"<< std::endl;
    std::cout << " 5 - Trigger: iniciar gravacao de eventos (.raw)" << std::endl;
    std::cout << " 6 - Start blink Led - Pino:"<< pinLed << std::endl;
    std::cout << " 7 - Stop blink Led" << std::endl;
    std::cout << " Q - Sair do programa "<< std::endl;
    std::cout << "     Digite a opção: ";
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



// Função quew efetua a cofniguração do barramento GPIO da Jetson, ela retorna um ponteiro tipo gpio_line usado para acessar os pinos de IO:
GPIO_Lines inicializaGPIO_Jetson(std::string chipIO, int active_line_trigger, int active_pin_trigger, int active_line_led, int active_pin_led, struct gpiod_chip **chip_ptr){

    GPIO_Lines lines = {nullptr, nullptr};

    //********************* Inicia configuração do barramenteo IO da Jetson: 
    // 1º. Abre o chip definido na string chipIO e armazena no ponteiro fornecido pelo main:
    *chip_ptr = gpiod_chip_open_by_name(chipIO.c_str());

    // struct gpiod_chip *chip = gpiod_chip_open_by_name(chipIO.c_str());
    if (!(*chip_ptr)) {
        perror("Erro ao abrir gpiochip0!!!");
        return lines;
    }

    // 2º. Captura o pino, line, para o TRIGGER:
    lines.trigger = gpiod_chip_get_line(*chip_ptr, active_line_trigger);
    if (!lines.trigger) {
        std::cerr << "Erro: Nao foi possivel obter a linha: " << active_line_trigger << std::endl;
        return lines;
    }
    gpiod_line_set_value(lines.trigger, 0);

    // 3º. Captura o pino, line, para piscar o LED:
    lines.led = gpiod_chip_get_line(*chip_ptr, active_line_led);
    if (!lines.led) {
        std::cerr << "Erro: Nao foi possivel obter a linha: " << active_line_led << std::endl;
        return lines;
    }
    gpiod_line_set_value(lines.led, 0);    

    
    // 4º. Faz um request da linha como SAÍDA para o trigger:
    if (gpiod_line_request_output(lines.trigger, "sync_trigger", 0) == 0){
        //Garantir que o pino do IO da Jetson inicie em nivel baixo, pois ele pode iniciar com um valor qualquer:
        gpiod_line_set_value(lines.trigger, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: trigger no pino: " << active_pin_trigger << std::endl;
        std::cout << "Jetson: Estado inicial do pino " << active_pin_trigger << "= 0V" << std::endl;
       // std::cout << "Jetson: trigger no SOC line: "<< active_line_trigger << std::endl;        
    } 
    else {
        perror("Erro ao configurar saída");
        return lines;
    }

     // 5º. Faz um request da linha como SAÍDA para o led:
    if (gpiod_line_request_output(lines.led, "sync_trigger", 0) == 0){
        //Garantir que o pino do IO da Jetson inicie em nivel baixo, pois ele pode iniciar com um valor qualquer:
        gpiod_line_set_value(lines.led, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Jetson: Led no pino: " << active_pin_led << std::endl;
        std::cout << "Jetson: Estado inicial do pino do led " << active_pin_led << "= 0V" << std::endl;   
    } 
    else {
        perror("Erro ao configurar saída");
        return lines;
    }   

    return lines;
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
int loadBiasesNaCamera(Metavision::Camera &cam) {

    // Os biases max e min. são definidos em https://docs.prophesee.ai/stable/hw/manuals/biases.html 
    // Os calores para a ca~mera SilkyEvCam pertencem a geração Gen3.1 VGA, assim os valores máximos e mínimo
    // são deinfidos como:
    
    int bias_diff_default= 299; // Não alterar o valor do bias_diff, o default é 299.
    
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

    std::string path= "../settings.json";

    // ANtes de gravar os valores de biases na camera verifica se os valores extrapolam os limites:
    int bias_diff_off= lerJsonFile(path, "bias_diff_off");
    if (bias_diff_off<bias_diff_off_min || bias_diff_off>bias_diff_off_max){
        std::cout<< "ERRO!! O valor de bias_diff_off= "<< bias_diff_off << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_diff_off_min << " e " << bias_diff_off_max << std::endl;  
        return -1;
    }

    int bias_diff_on= lerJsonFile(path, "bias_diff_on");
    if (bias_diff_on<bias_diff_on_min || bias_diff_on>bias_diff_on_max){
        std::cout<< "ERRO!! O valor de bias_diff_on= "<< bias_diff_on << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_diff_on_min << " e " << bias_diff_on_max << std::endl;  
        return -1;
    }

    int bias_fo= lerJsonFile(path, "bias_fo");
    if (bias_fo<bias_fo_min || bias_fo>bias_fo_max){
        std::cout<< "ERRO!! O valor de bias_fo= "<< bias_fo << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_fo_min << " e " << bias_fo_max << std::endl;  
        return -1;
    }   
    
    int bias_hpf= lerJsonFile(path, "bias_hpf");
    if (bias_hpf<bias_hpf_min || bias_hpf>bias_hpf_max){
        std::cout<< "ERRO!! O valor de bias_hpf= "<< bias_hpf << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_hpf_min << " e " << bias_hpf_max << std::endl;
        return -1;  
    }   

    int bias_refr= lerJsonFile(path, "bias_refr");
    if (bias_refr<bias_refr_min || bias_refr>bias_refr_max){
        std::cout<< "ERRO!! O valor de bias_refr= "<< bias_refr << " está fora dos limites!!"<< " Ele deve ser entre=" << bias_refr_min << " e " << bias_refr_max << std::endl; 
        return -1; 
    }  

    // Se todos os valores forma coerentes, os biases serão gravados na câmera:
    cam.load(path.c_str());
    std::cerr << "Carregando arquivo de configuração: " << path.c_str() << std::endl;
    return 0;
}



// Funcção principal:
int main(int argc, char *argv[]) {
    // Rotina que limpa o terminal:
    limparTela();

    // Flag que habilita ou não o menu, apenas para testes:
    bool hab_exibe_menu= true;  
    
    // Flaque que inicializa o loop while:
    bool running = true; 
    
    //********************************************************************************************/
    //****************************** Inicia COnfiguração da Jetson *******************************/

    //Offsets para acesar os IOs:
    int64_t duracao_PulsoTrigger_microSeg= 30000; // em micro segundos
    int64_t duracao_PreTrigger_microSeg= 50000; // em micro segundos
    int64_t duracao_PosTrigger_microSeg= 50000; // em micro segundos

    int64_t duracao_PulsoLed_microSeg= 200000; // em micro segundos;

    //Seleção de pinos de IOs disponíveis
    int lineA= 43;  // GPIO13 PH.00 - Pino IO no Header= 33
    int lineB= 85;  // GPIO12 PN.01 - Pino IO no Header= 15
    int lineC= 105; // GPIO01 PQ.05 - Pino IO no Header= 29
    int lineD= 144; // GPIO01 PAC.06 - Pino IO no Header= 7

    // N] do pino físico no barramento de GPIO da Jetson:
    int header_pinA= 33;
    int header_pinB= 15;
    int header_pinC= 29;
    int header_pinD= 7;

    int active_pin_trigger= header_pinD;
    int active_pin_led= header_pinA;

    // String que contem o chip usado SOI da Jetson:
    std::string chipIO= "gpiochip0";

    
    // Declara o ponteiro do chip aqui para poder ser fechado depois dentro do main:
    struct gpiod_chip *chip = nullptr;

    // Chama função "inicializaGPIO_Jetson" para configuração do barramento GPIO da Jetson.
    // Primeiro gerando  lineTrigger para o trogger da camera:
    GPIO_Lines gpios= inicializaGPIO_Jetson(chipIO.c_str(), lineD, header_pinD, lineA, header_pinA, &chip);


    // Verificação se os pinod forma configurados ok, por segurança:
    if (gpios.trigger == nullptr || gpios.led == nullptr) {
        std::cerr << "Falha crítica na inicialização dos GPIOs. Abortando!!!!" << std::endl;
        if (chip) gpiod_chip_close(chip);
        return 1;
    }



    //********************************************************************************************/
    //*************************** Inicia COnfiguração da camera de eventos ***********************/

    // Booleano que define se a câmera usada é HD ou VGA:    
    bool sensorHD= true;    

    // 1- Instancia um objeto "camera" da classe "Metavision::camera":
    Metavision::Camera camera;
    try {
        camera = Metavision::Camera::from_first_available();
        //std::cout << "Instanciado o objeto camera." << std::endl;
    } 
    catch (const Metavision::CameraException &e) {
        std::cerr << "Erro ao abrir a camera: " << e.what() << std::endl;
        return 1;
    }

    // Variáveis que guardam os dados da camera:
    std::string serial_cam_1= "unknow";
    std::string versionFirm= "unknow";
    std::string dataEncodeFormat= "unknow";
    std::string plugin= "unknow";
    std::string fabricante= "unknow";


    // Captura o numero de seria da camera instanciada:
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
    if (loadBiasesNaCamera(camera))
        std::cout << "ERRO!! Biases não gravado na câmera." << std::endl;
    else
        std::cout << "Câmera configurada com os biases do .json." << std::endl;


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
    gpiod_line_set_value(gpios.trigger, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpiod_line_set_value(gpios.trigger, 0);

    // Aguarda o sistema processar esse evento interno antes de liberar o menu
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


    std::cout << "Camera inicializada" << std::endl;

    // Define a variável que contém o tempo de duração que o LEd irá píscar:
    int duracao_PulsoLed_miliSeg= 20; // em micro segundos;

    // Instancia o objeto meuLed da classe LEDCotroller: 
    // Cria classe com os parametros inciais lineLed e Duração_Pulso_miliseg:
    LEDController meuLed(gpios.led, duracao_PulsoLed_miliSeg);


    //********************************************************************************************  
    //****************************** Loop principal: *********************************************
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
            showMenu(active_pin_trigger, active_pin_led, duracao_PulsoTrigger_microSeg);
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
                        camera.stop();
                        loadBiasesNaCamera(camera);
                        camera.start();
                        break;
                /*
                        {
                            std::cout << std::endl;
                            if (loadBiasesNaCamera(camera)==-1)
                                std::cout<< "ERROR!! Não gravou os biases na câmera.";
                            else
                                std::cout<< "Biases gravados na câmera.";
                            break;
                        }  */

                case '3':
                        {
                            gpiod_line_set_value(gpios.trigger, 1);                           
                            std::cout << " Pino " << active_pin_trigger << " em nivel ALTO" << std::endl;
                            break;
                        }

                case '4':
                        {
                            gpiod_line_set_value(gpios.trigger, 0);
                            std::cout << " Pino " << active_pin_trigger << " em nivel BAIXO" << std::endl;
                            break;
                        }    

                case '5':
                        {
                            std::cout << "\nTrigger Com duração de: "<< duracao_PulsoTrigger_microSeg << "us" << std::endl;
            
                            // Passando o objeto camera, o ponteiro da linha GPIO e a duração do pulso trigger
                            std::thread t(saveDataFileWithTrigger, std::ref(camera), gpios.trigger, duracao_PulsoTrigger_microSeg, duracao_PreTrigger_microSeg, duracao_PosTrigger_microSeg, serial_cam_1);
                            
                            // Desacoplar a thread para que o viewer não trave
                            t.detach();
                            break;
                        } 

                case '6':
                         meuLed.start(); // Sem parâmetros aqui, o objeto já sabe o que fazer
                         break;                        
                        
                case '7':
                         meuLed.stop(); // Sem parâmetros aqui, o objeto já sabe o que fazer
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

    // Fecha o pino da Jetson
    if (gpios.trigger) 
        gpiod_line_release(gpios.trigger);
    
    // Fecha o pino da Jetson
    if (gpios.led) 
        gpiod_line_release(gpios.led);
    

    // LIbera o chip reservado na Jetson:    
    if (chip)
        gpiod_chip_close(chip);

    // Fecha todas as janelas:    
    cv::destroyAllWindows();

    return 0;
}