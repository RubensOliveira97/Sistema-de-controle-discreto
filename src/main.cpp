#include <Arduino.h>
#include <VarSpeedServo.h>
#include <Ultrasonic.h>

#define pino_trigger 4
#define pino_echo 5
#define num 30

VarSpeedServo servo1;
Ultrasonic ultrasonic(pino_trigger, pino_echo);
bool teste;
//=====================Variáveis de armazenamento de distância============================================
float values[num]; //vetor com num posições, armazena os valores para cálculo da média móvel
float distancia;
float distanciaCorrigida;
float tempoAnterior;
float tempoAtual;
float tempoInicial;

//=====================Variáveis de Controle=============================================================
float setPoint = 0;
float erro;
float pidProporcional;
float pidIntegrador;
float pidDerivativo;
float kP=1;
float kI;
float kD;
float PID;
float servoMax = 180;
float servoMin = 0;
float moving_average(float sig);
//========================================================================================================
void setup()
{ tempoInicial=millis();
  servo1.attach(9);
  Serial.begin(9600);
  pinMode(7, INPUT_PULLUP);
}

float calculoProporcional()
{

  erro = distanciaCorrigida - setPoint;
  pidProporcional = erro*kP;
  return pidProporcional;
}

float leituraDistancia()
{
  float cmMsec;
  long microsec = ultrasonic.timing();
  cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  return cmMsec;
}

float moving_average(float sig)
{
  int i;         //variável auxiliar para iterações
  float acc = 0; //acumulador

  //Desloca o vetor completamente eliminando o valor mais antigo
  for (i = num; i > 0; i--)
    values[i] = values[i - 1];

  values[0] = sig; //carrega o sinal no primeiro elemento do vetor

  // long sum = 0;            //Variável para somatório

  for (i = 0; i < num; i++)
    acc += values[i];

  return acc / num; //Retorna a média móvel
}

void loop()
{if(digitalRead(7)){
setPoint = 17;
  float leitura = leituraDistancia();
  distancia = leitura <= 42 ? leitura : distancia;
    if (distancia < 42)
    {
      distanciaCorrigida = moving_average(distancia);
    }
    
    calculoProporcional();

      if (pidProporcional >= servoMax)
      {
        pidProporcional = servoMin;
      }
      // else if (-pidProporcional <= servoMin)
      // {
      //   pidProporcional = servoMax;
      // }

      if (pidProporcional > 0)
      {
        servo1.write(-pidProporcional+90, 0, true); //Giro horario
      }
      else if (pidProporcional < 0)
      {
        servo1.write(-pidProporcional+90, 0, true); //Giro antihorario
      }
}

   Serial.print(millis());
    Serial.print(" ");
    Serial.print(setPoint);
    Serial.print(" "); 
    if(!digitalRead(7)){ 
      Serial.println(0);
    }else{
    Serial.println(distanciaCorrigida);
    }
    
}
//   delay(50);
//   //========================MEDIA DOS VALORES DOS SENSORES========================================================================================================
//   //Média dos valores do sensores para filtrar o ruido e diminuir oscilações

//   //========================================CÁLCULO DA PROPORCIONAL==================================================================================================

//   erro = vSen - setPoint; //Calculo do erro entre o Sensor e o Setpoint
//   vPID_P = kp * erro;     //Cálculo da proporcional

//   //========================================CÁLCULO DA INTEGRAL======================================================================================================
//   //A parte integral só deve atuar se estiver dentro do limiar

//   if (limiar < erro < limiar)
//   {
//     vPID_I = vPID_I + (ki * erro); //Cálculo da integral
//   }

//   //========================================CÁLCULO DA DERIVADA=======================================================================================================

//   tempoAnterior = time;                         //Tempo anterior é armazenado antes de ler o atual
//   time = millis();                              //Leitura do tempo atual
//   tempoPassado = (time - tempoAnterior) / 1000; //Milisegundos -> segundos

//   vPID_D = kd * ((erro - erroAnterior) / tempoPassado); //Cálculo da derivada

//   //========================================CÁLCULO PID===============================================================================================================

//   PID = vPID_P + vPID_I + vPID_D;

//   //========================================RESTRIÇÕES SUPERIORES E INFERIORES========================================================================================

//   if (PID < -PWMmax)
//   {
//     PID = -PWMmax;
//   }
//   if (PID > PWMmax)
//   {
//     PID = PWMmax;
//   }

//   //=====================================================ATUAÇÃO=======================================================================================================

//   if (PID > 0)
//   {
//     analogWrite(PWM1, PID); //Motor 1 (vermelho e preto)
//     analogWrite(PWM2, PID); //Motor 2 (branco e preto)
//     digitalWrite(S1, LOW);
//     digitalWrite(S2, HIGH);
//     digitalWrite(S3, LOW);
//     digitalWrite(S4, HIGH); //Giro horario
//   }
//   else
//   {
//     analogWrite(PWM2, -PID); //Motor 1 (vermelho e preto)
//     analogWrite(PWM1, -PID); //Motor 2 (branco e preto)
//     digitalWrite(S1, HIGH);
//     digitalWrite(S2, LOW);
//     digitalWrite(S3, HIGH);
//     digitalWrite(S4, LOW); //Giro antihorario
//   }
//   //=====================================================ATUALIZA AS VARIAVEIS==========================================================================================

//   erroAnterior = erro; //Armazena o erro anterior.
//   MPot = 0;
//   MSen = 0; //Zera valores das médias

//   //==============================PLOTAGEM==========================================================================================================================

//   Serial.print(vSen);
//   Serial.print(",");
//   Serial.println(setPoint);
// }

// //FIM