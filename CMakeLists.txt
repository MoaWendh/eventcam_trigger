
cmake_minimum_required(VERSION 3.5)

project(metavision_trigger_viewer)

# Define o padrão C++17 exigido pelo Metavision SDK
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Busca de Dependências
# Metavision SDK (Componentes core e stream para câmera e gerador de frames)
find_package(MetavisionSDK COMPONENTS core stream REQUIRED)

# Boost (Exigido internamente pelo Metavision)
find_package(Boost COMPONENTS program_options REQUIRED)
add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)

# OpenCV (Para a janela gráfica e processamento de frames)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)

# Libgpiod (Para o controle dos pinos da Jetson)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# 2. Configuração de Inclusões
# Adiciona os diretórios de inclusão encontrados e o caminho específico do OpenCV 4 na Jetson
include_directories(
    ${MetavisionSDK_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GPIOD_INCLUDE_DIRS}
    /usr/include/opencv4
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 3. Definição do Executável
set(SOURCE_FILE event_trigger.cpp controlIO.cpp) # Ajuste para o nome real do seu arquivo .cpp
add_executable(event_trigger ${SOURCE_FILE})

# 4. Linkagem de Bibliotecas
target_link_libraries(event_trigger
    PRIVATE
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
    ${GPIOD_LIBRARIES}
    Boost::program_options
)

# Mensagens de status para conferência durante o cmake ..
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")