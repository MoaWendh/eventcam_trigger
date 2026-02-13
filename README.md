### Trigger de hardware da SilkyEvCam através do IO da Jetson Orin Nano
Projeto: Voris  
Data: 13/02/2026  
Autor: Moacir Wendhausen  
SDK Metavision: 5.1.1

---
### Características da SilkyEvCam  

    Available Data Encoding Formats                   EVT3
    Connection                                        USB
    Current Data Encoding Format                      EVT3
    FW Build Date                                     Sun Oct 30 23:37:15 2022
    FW Release Version                                3.9.0-C
    FW Speed                                          5000
    Integrator                                        CenturyArks
    Sensor Name                                       Gen3.1
    Serial                                            00000680
    System Version                                    4.2.0

------

**Principais características do programa:**  
  
1- Visualização gráfica dos eventos em tempo real:  

![alt text](image-2.png)  

2- Menu de opções para testar o trigger nos pinos da Jetson: 

![alt text](image.png)

3- Salva dados da camera de eventos através de **thread** sincronizada com **trigger de hardware** com duração variável em milisegundos.