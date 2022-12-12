# Projeto-Gangorra
Projeto das disciplinas de Sistamas Embarcados e Instrumentação

### embedded_software  
Branch master para armazenar o historico de versão do software embarcado 

#### Documentação
https://www.notion.so/73943221b94847fca581e2f51ed079da?v=368dd51d757e429dbad238efdc53bc0f

### Estrutura 
-> **Master** => branch responsável por armazenar o versionamento dos arquivos da estrutura mecânica do projeto 

-> **embedded_software** => branch responsável por armazenar o versionamento dos arquivos do software embarcado na esp-32 

-> **app_mobile** =>  branch responsável por armazenar o versionamento dos arquivos do software mobile 

# Tutorial workflow das branchs

Digamos que vc esteja trabalhando com o software embarcado, onde há varias integrações com sensores e atuadores. Temos a branch principal para o software embarcado que é a branch **embedded_software**. Vc ficou com a tarefa de integrar o sensor ultrassônico, como prosseguir?

Crie uma branch que seja uma cópia da branch principal, isto é, inclui inclusive os commits anteriores. Para isso, digite 
```
git checkout -b feature/<nome_da_task> embedded_software
```

A nova branch será criada e a branch corrente no seu terminal será a nova branch. 

Agora faça suas implementações sem receios. Adicione modificações, commits e push a sua branch.

## Merge branchs

Quando quiser fazer o merge de duas branchs, primeiro rode o seguinte comando para a branch a qual será feito o merge da branch secundária. Suponha que temos a branch para o sensor ultrassônico, feature/ultrasonic_sensor.
```
git checkout embedded_software
git merge feature/ultrasonic_sensor
```

Isto irá fazer o merge da branch feature/ultrasonic_sensor em embedded_software. Porém, essas alterações serão realizadas apenas localmente.

Para fazer o merge no remote repository, isto é, repositório remoto do git, faça:
```
git push --set-upstream origin <branch name>
```

## Utilizar o mesmo esquema para bugfixes.