# Documentação do Software embarcado

Created: November 14, 2022 2:10 PM
Last Edited Time: November 23, 2022 6:43 PM
Stakeholders: Anonymous
Status: In Review
Type: Technical Spec

# Sumário

Para que o sistema da Gangorra de Demonstração de Inclinação seja possível, devem ser implementados meios de realizar medições de variáveis do ambiente do sistema, através de sensores e realizar o controle de atuadores responsáveis por manter a haste equilibrada, desta maneira, o firmware é o coração do sistema.

# Background

- Tornar o sistema mais eficiente ao adicionar um Sistema de Tempo-Real (RTOS)
- Implementações de Tasks do sistema
- Leitura dos sensores em tasks do sistema
- Controle PWM dos atuadores com tasks do sistema

# Objetivos

- Sistema capaz de escalonar as tasks
- Sistema responsivo
- Percepção da inclinação da haste da gangorra através dos sensores
- Controle dos atuadores
- Interface de usuário (Display LCD, LEDs)

### Soluções Propostas

- Utilização do FreeRTOS para controle das tasks do sistema.
- Devido a aplicação de um Sistema de Tempo-Real, o sistema tende a ser mais eficiente e responsivo.
- Os sensores giroscópio/acelerômetro e sensores ultrassônicos foram previamente testados individualmente, e através da integração com o firmware estes devem ser atribuídos a tasks do sistema.
- O PWM para o acionamento dos atuadores do sistema foram testados individualmente, e através da integração com o firmware espera-se que eles sejam atribuídos a tasks do sistema.
- O display LCD, assim como os LEDs foram testados individualmente e espera-se que estes sejam atribuídos a tasks do sistema.

### Riscos

- Os sensores ultrassônicos podem acabar causando interferências, cada sensor está em uma extremidade da gangorra, entretanto, por emitirem ondas sonoras através do *trigger*   pode ocorrer choque entre as ondas dos dois sensores.
- Um dos pinos utilizados pelo sensor LCD, o pino **GPIO_0** produz um sinal de PWM durante o boot da ESP 32, conforme a documentação da pinout da placa, portanto, pode causar bug na saída do display durante o boot do sistema, levamos em consideração trocar este pino, porém adiciona um trabalho maior ao ter que fabricar uma nova PCB apenas para corrigir este problema.

# Informações a respeito da implementação do firmware podem ser obtidas na Wiki a seguir:

[Wiki - Firmware](Documentac%CC%A7a%CC%83o%20do%20Software%20embarcado%20cb58845ba6ef4e04be6b9cc53eff0f24/Wiki%20-%20Firmware%20dc74ac75875b44fe8230f22a48d59821.md)