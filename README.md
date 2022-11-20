# Projeto-Gangorra
Projeto das disciplinas de Sistamas Embarcados e Instrumentação

### embedded_software  
Branch master para armazenar o historico de versão do software embarcado 

#### Documentação
https://www.notion.so/73943221b94847fca581e2f51ed079da?v=368dd51d757e429dbad238efdc53bc0f

------------

├── README.md          <- The top-level README for developers using this project.
│
│
├── main    <- Pasta principal do firmware
|    ├── components         <- Libs p/ integração com os componentes.
|    |   ├── delay      
|    |   └── 
|    |   └── lcd
|    |   └── mpu6050
|    |   └── ultrasonic
|    |
│    ├── inc              <- Libs do firmware.
|    |   └── bluetooth.hpp <- biblioteca bluetooth
|    |   └── display.hpp <- biblioteca display
|    |   └── motors.hpp <- biblioteca motores
|    |   └── sensores.hpp <- biblioteca sensores
|    |   └── system.hpp <- biblioteca sistema
|    |
│    ├── src              <- Código principal do firmware.
|    |   └── bluetooth.hpp <- bluetooth
|    |   └── display.hpp <- display
|    |   └── motors.hpp <- motores
|    |   └── sensores.hpp <- sensores
|    |   └── system.hpp <- sistema
|    |   
│
├── CMakeList.txt   <- CMakeList para a compilação do projeto.
|
├── docs             <- Documentação em formato PlantUML
│    ├── out              <- Código principal do firmware.
|    |    └── components <- view components da documentação
|    |    └── containers <- view containers da documentação
|    |    └── contexto   <- view contexto da documentação
