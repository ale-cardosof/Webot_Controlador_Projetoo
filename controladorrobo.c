/*
 * Descrição: Atividade IA
 * Autor: Alexandre Cardoso Feitosa
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 256
#define NumSensoresProx 8
#define NumDeLeds 10
#define QtdAceleracao 5


int main(int argc, char** argv)
{

    int i = 0, qntdCaixasEncontradas = 0;
    double leituraSensorProx[NumSensoresProx]; // Vetor de leitura dos sensores
    double AceleradorEsquerdo=1, AceleradorDireito=1; // Acelerador do robô
    char texto[256]; // Variável para printar
    double mediaAceleracao[QtdAceleracao]; // Vetor p/ Calcular a média das acelerações
    bool deteccaoDeCaixaMovel = false;
    
    // Zera vetor de print
    for(i=0;i<257;i++) texto[i]='0';
    
    // Inicializiação do ambiente
    wb_robot_init();
    
    // ------------------- Configuração do supervisor
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("ePuck"); //captura o supervisor
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation"); //identifica o campo de posição
    const double *posicao; // recebe a posição do robo no mundo
    
    // ------------------- Configuração dos motores
    WbDeviceTag MotorEsquerdo, MotorDireito;
    // Atribui os motores
    MotorEsquerdo = wb_robot_get_device("left wheel motor");
    MotorDireito = wb_robot_get_device("right wheel motor");
    // Define a posição do motor
    wb_motor_set_position(MotorEsquerdo, INFINITY);
    wb_motor_set_position(MotorDireito, INFINITY);
    // Define velocidade 0 para cada Motor
    wb_motor_set_velocity(MotorEsquerdo, 0);
    wb_motor_set_velocity(MotorDireito, 0);
    
    // ------------------- Configuração do Acelerometro
    WbDeviceTag Acelerometro;
    // Atribui o Acelerometro
    Acelerometro = wb_robot_get_device("accelerometer");
    // Ativa o Acelerometro
    wb_accelerometer_enable(Acelerometro, 10);
    const double* values = wb_accelerometer_get_values(Acelerometro);
    
    // ------------------- Configuração dos Sensores
    WbDeviceTag SensorProx[NumSensoresProx];
    // Atribui os sensores
    SensorProx[0] = wb_robot_get_device("ps0");
    SensorProx[1] = wb_robot_get_device("ps1");
    SensorProx[2] = wb_robot_get_device("ps2");
    SensorProx[3] = wb_robot_get_device("ps3");
    SensorProx[4] = wb_robot_get_device("ps4");
    SensorProx[5] = wb_robot_get_device("ps5");
    SensorProx[6] = wb_robot_get_device("ps6");
    SensorProx[7] = wb_robot_get_device("ps7");
    // Habilita todos os sensores
    wb_distance_sensor_enable(SensorProx[0], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[1], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[2], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[3], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[4], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[5], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[6], TIME_STEP);
    wb_distance_sensor_enable(SensorProx[7], TIME_STEP);
    
    // ------------------- Configuração dos Leds
    WbDeviceTag Leds[NumDeLeds];
    // Atribui os leds
    Leds[0] = wb_robot_get_device("led0");
    Leds[1] = wb_robot_get_device("led1");
    Leds[2] = wb_robot_get_device("led2");
    Leds[3] = wb_robot_get_device("led3");
    Leds[4] = wb_robot_get_device("led4");
    Leds[5] = wb_robot_get_device("led5");
    Leds[6] = wb_robot_get_device("led6");
    Leds[7] = wb_robot_get_device("led7");
    Leds[8] = wb_robot_get_device("led8");
    Leds[9] = wb_robot_get_device("led9");
    
    // Define todos os Leds como desligados
    wb_led_set(Leds[0], 0);
    wb_led_set(Leds[1], 0);
    wb_led_set(Leds[2], 0);
    wb_led_set(Leds[3], 0);
    wb_led_set(Leds[4], 0);
    wb_led_set(Leds[5], 0);
    wb_led_set(Leds[6], 0);
    wb_led_set(Leds[7], 0);
    wb_led_set(Leds[8], 0);
    wb_led_set(Leds[9], 0);
    
    /* Atualização */
    int iteracoes = 0; // Soma quantas iterações passaram
    while (wb_robot_step(TIME_STEP) != -1){
        for(i=0;i<256;i++) texto[i]=0;
        
        // Le e exibe os valores dos sensores
        for(i=0;i<NumSensoresProx;i++){
          leituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]);
          sprintf(texto,"%s|%d: %5.2f  ",texto,i,leituraSensorProx[i]);
        }
        printf("%s\n",texto);
        
        posicao = wb_supervisor_field_get_sf_vec3f(trans_field);
        
        // Caso o robô esteja próximo da caixa móvel
        if (posicao[0] > -0.65 && posicao[0] < -0.3 && posicao[2] > -0.65 && posicao[2] < -0.3){
          AceleradorDireito = 1;
          AceleradorEsquerdo = 1;
          // Se algum sensor for maior que 1000, pisca 3 vezes o sensor e soma um
          if (leituraSensorProx[0] > 500 || leituraSensorProx[1] > 500 || leituraSensorProx[6] > 500 || leituraSensorProx[7] > 500){
            for(i = 0; i < 3; i++){
                // Liga os leds e aguarda um periodo de 200ms
                wb_led_set(Leds[0], 1);
                wb_led_set(Leds[1], 1);
                wb_led_set(Leds[2], 1);
                wb_led_set(Leds[3], 1);
                wb_led_set(Leds[4], 1);
                wb_led_set(Leds[5], 1);
                wb_led_set(Leds[6], 1);
                wb_led_set(Leds[7], 1);
                wb_led_set(Leds[8], 1);
                wb_led_set(Leds[9], 1);
                wb_robot_step(200);
                
                // Apaga os leds e aguarda um periodo de 200ms
                wb_led_set(Leds[0], 0);
                wb_led_set(Leds[1], 0);
                wb_led_set(Leds[2], 0);
                wb_led_set(Leds[3], 0);
                wb_led_set(Leds[4], 0);
                wb_led_set(Leds[5], 0);
                wb_led_set(Leds[6], 0);
                wb_led_set(Leds[7], 0);
                wb_led_set(Leds[8], 0);
                wb_led_set(Leds[9], 0);
                wb_robot_step(200);  
            }
            qntdCaixasEncontradas++;
            deteccaoDeCaixaMovel = true;
            printf("Caixas detectadas: %d.\n", qntdCaixasEncontradas);
            
            // Mantém ligados a quantidade de leds correspondentes as caixas detectadas
            for(i = 0; i < qntdCaixasEncontradas;i++){
              wb_led_set(Leds[i], 1);
            }
           }
          }else if (leituraSensorProx[0] > 500){
              AceleradorDireito = 1;
              AceleradorEsquerdo = -0.1;
          }else if (leituraSensorProx[1] > 400){
              AceleradorDireito = 1;
              AceleradorEsquerdo = -0.2;
          }else if (leituraSensorProx[2] > 400){
              AceleradorDireito = 1;
              AceleradorEsquerdo = -0.3;
          }else if (leituraSensorProx[7] > 400){
              AceleradorDireito = -0.1;
              AceleradorEsquerdo = 1;
          }else if (leituraSensorProx[6] > 400){
              AceleradorDireito = -0.2;
              AceleradorEsquerdo = 1;
          }else if (leituraSensorProx[5] > 400){
              AceleradorDireito = -0.3;
              AceleradorEsquerdo = 1;
          }else{
              AceleradorDireito = 1;
              AceleradorEsquerdo = 1;
          }
        
        wb_motor_set_velocity(MotorEsquerdo, 4.28 * AceleradorEsquerdo);
        wb_motor_set_velocity(MotorDireito, 4.28 * AceleradorDireito);

        iteracoes++;
    
    };

    wb_robot_cleanup();
 
    
    return 0;

}
