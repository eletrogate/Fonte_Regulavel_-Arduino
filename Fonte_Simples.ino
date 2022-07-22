#include <PID_v1.h> // Inclusao da biblioteca do controlador

float VccMedido = 5; //A tensão medida no pino de 5V. Caso voce nao possua um multimetro, deixe 5.
float setpoint = 0;

int PIN_INPUT = A0;  // Pino de entrada do controlador PID, que irá servir como sensor
int PIN_OUTPUT = 5; // Pino de saída, que irá controlar o sistema

char resposta[25] = {"         "}; // String que recebe os dados seriais

double Setpoint, Input, Output, kp = 0, ki = 1, kd = 0; // Kp, Ki e Kd são os ajustes do controlador PID.

PID controlador (& Input, & Output, & Setpoint, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(PIN_INPUT, INPUT); // Inicializando o pino do sensor do controlador como entrada
  pinMode(PIN_OUTPUT, OUTPUT); // Inicializando como saída o pino do controlador da tensão.
  controlador.SetMode(AUTOMATIC); // Configura o controlador para funcionar em modo automático
  controlador.SetSampleTime(0); // Seta o controlador para ter a taxa de amostragem mais rápida possível.
  Serial.println("CLEARDATA");  // Limpa os dados das legendas do Plotter Serial
  Serial.println("Setpoint, Zero, Saida"); // Insere as legendas no Plotter Serial
}

void loop() {
  float Adc = VccMedido / 1023; //  ADC é a relação entre a tensão e os níveis ADC de 0 a 1023
  EscutaSerial(); // A função que irá ficar escutando a comunicação serial e irá inserir esses dados no vetor resposta
  Setpoint = setpoint / Adc; //Converte o Setpoint de Volts para níveis ADC (que é o que a placa entende)
  Input = analogRead(PIN_INPUT); //Carregando o valor do pino analógico para o controlador
  controlador.Compute(); // O controlador calcula qual deve ser a saída do pino PWM, para tudo ocorrer certo.
  analogWrite(PIN_OUTPUT, Output); //O pino de saída envia o sinal PWM que foi computado.
  imprimir(setpoint, 0,analogRead(A0)*Adc); // A função imprimir é um amontoado de "Serial.print"", só fizemos ela para facilitar.

}

void imprimir(float valor1, float valor2, float valor3) // A função recebe 3 parametros, e já imprime eles no formato correto.
{
  static int linha = 0;
  Serial.print(valor1);
  Serial.print(",");
  Serial.print(valor2);
  Serial.print(",");
  Serial.println(valor3);
}

void EscutaSerial() { // Função que recebe os dados do Plotter Serial e converte em um valor de Setpoint
  if (Serial.available() > 0) {
    byte leitura, i = 0;
    strcpy(resposta, "                  ");
    leitura = 255;
    while (leitura != 10) {
      if (Serial.available() > 0) {
        leitura = Serial.read();
        resposta[i] = (char)leitura;
        i++;
      }
    }
    i = 0;
  }
  setpoint = atof(strtok(resposta, " "));
}