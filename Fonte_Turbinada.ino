/* INCLUSÃO DAS BIBLIOTECAS */
#include <PID_v1.h> // Inserindo a biblioteca para realizar o controle da tensão
#include <Wire.h>   // Inserindo a biblioteca para realizar a comunicação I2C                     
#include <Adafruit_GFX.h> // Biblioteca de efeitos visuais
#include <Adafruit_SSD1306.h> // Biblioteca do módulo OLED utilizado.

/* DEFINIÇÃO DAS VARIÁVEIS E CONSTANTES RESPONSÁVEIS PELA CONFIGURAÇÃO*/
#define VccMax 5  // Definindo qual a tensão máxima que o módulo pode regular
#define Qtd_Amostras_3v3 100    // Quantidade de amostras do filtro (ver o post sobre filtros)
#define Intervalo_Amostragem 1 // definindo o intervalo de amostragem em ms.
float Calibracao3v3 = 3.3;    // A tensão medida no pino de 3.3V. IMPORTANTE: Caso você não tenha um multimetro para medir, deixe o valor em 3.3V.

int Leitura_3v3 = 0; // Variavel global que é chamada em várias funções e faz a leitura em níveis ADC da tensão de 3.3V.
int PIN_INPUT = A0, PIN_OUTPUT = 5; // Declaração dos pinos de saida e entrada do controlador.

double Setpoint, Input, Output, // Cariáveis que serão computadads no controlador
       kp = 0, ki = 1, kd = 0;  // Os parametros do controlador PID.

#define SCREEN_WIDTH 128                  // OLED display largura, em pixels
#define SCREEN_HEIGHT 32                  // OLED display altura, em pixels

#define OLED_RESET -1                     // Pino de reset

/* INSERINDO OS OBJETOS PARA REALIZAR O CONTROLE DO DISPLAY E DO CONTROLADOR*/
PID controlador (& Input, & Output, & Setpoint, kp, ki, kd, DIRECT); // Declaração do objeto controlador PID.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);   //Criando o objeto para controlar o display

/*CONFIGURAÇÃO DAS VARIÁVEIS PARA CONTROLAR O ENCODER*/
static int pinA = 2;                      //Pino para interrupção interna do Enconder
static int pinB = 3;                      //Pino para interrupção interna do Enconder
volatile byte aFlag = 0;                  //PinA detectando apenas a borda de subida
volatile byte bFlag = 0;                  //PinB como borda de subida na detecção
volatile int8_t encoderPos = 0;             //Posição atual do Encoder, que pode variar de 0 a 9
volatile byte prevEncoderPos = 0;         //Variavel para armazenar o valor anterior, para saber se o encoder foi girado.
volatile byte reading = 0;                //Armazena o valor dos pinos de interrupção

const byte buttonPin = 4;                 //Pino do botão do encoder
byte oldButtonState = HIGH;               //Armazena o antigo estado do botão
const unsigned long debounceTime = 10;    //Delay para evitar erros na leitura do botão
unsigned long buttonPressTime;            //Tempo que o botão foi pressionado para debounce

void setup() {
  Serial.begin(9600); // Iniciando a comunicacao serial para ver a tensão no Plotter Serial
  pinMode(PIN_INPUT, INPUT); // Inicializando o pino do sensor do controlador como entrada
  pinMode(PIN_OUTPUT, OUTPUT); // Inicializando como saída o pino do controlador da tensão.

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))     //Inicia o display
  {
    Serial.println(F("SSD1306 a inicialização falhou"));   //Verifica se a conexão com o display falhou
    for (;;);                                         //Caso falhou, vai entrar em um loop infinito para não continuar
  }
  display.clearDisplay();      //A inicialização do display foi bem sucedida, então limpa o display
  display.setTextColor(SSD1306_WHITE);                //Setando a cor do texto para branco

  pinMode(pinA, INPUT_PULLUP);                        //Seta PinA como entrada PULLUP,para não precisar do resistor.
  pinMode(pinB, INPUT_PULLUP);                        //Seta PinB como entrada PULLUP,para não precisar do resistor.
  attachInterrupt(0, PinA, RISING);                   //Inicia a rotina de interrupção dos pinos de rotação do encoder
  attachInterrupt(1, PinB, RISING);
  pinMode (buttonPin, INPUT_PULLUP);                  //Seta o botão como entrada PULLUP,para não precisar do resistor.

  controlador.SetMode(AUTOMATIC); // Configura o controlador para funcionar em modo automático
  controlador.SetSampleTime(0); // Seta o controlador para ter a taxa de amostragem mais rápida possível.
  Serial.println("CLEARDATA"); // Limpa os dados das legendas do Plotter Serial
  Serial.println("Setpoint, Zero, Saida"); // Insere as legendas no Plotter Serial
  Leitura_3v3 = analogRead(A1);  // Leitura do valor ADC referente a 3.3V, onde aqui é o valor bruto que ainda será filtrado
  Amostragem(); // Chamando a função que atualiza o valor do filtro.
}


void loop() {

  float setpoint = 0, Adc = 0; // Setpoint recebe o valor que deve ser ajustado na saída, e ADC qual é a relação entre a tensão e os níveis ADC de 0 a 1023
  setpoint = Tensao(); // Tensao é a função responsável por converter o valor selecionado no display em um valor de tensão float
  Adc = Calibracao3v3 / filtro3v3(0); // Essa função ADC vai fazer a relação entre s níveis ADC e tensão, dividindo o valor da calibração pelo nível ADC filtrado
  // É importante ser filtrado porque não haverá qaquel
  Leitura_3v3 = analogRead(A1);  // Faz a leitura de 3.3V para atualizar no filtro.

  Amostragem(); // Essa é a função que fará a amostragem no tempo que determinamos no intervalo de amostragem que definimos na segunda linha de código.

  Setpoint = (int)(setpoint / Adc); // Converte o valor do setpoint (minusculo, que é em volts) em níveis ADC, pela relação feita pela variável 'Adc'

  Input = analogRead(PIN_INPUT); //O controlador lendo a entrada da porta analógica, que é a saída do transistor
  controlador.Compute(); // O controlador calcula qual deve ser a saída do pino PWM, para tudo ocorrer certo.
  analogWrite(PIN_OUTPUT, Output); //O pino de saída envia o sinal PWM que foi computado.
  imprimir(setpoint, 0, analogRead(A0)*Adc); // A função imprimir é um amontoado de "Serial.print"", só fizemos ela para facilitar.
}

void Amostragem() { // Essa função verifica se o tempo de amostragem  selecionado ocorreu
  static unsigned long timer1 = 0; // A variável que irá contar o útimo
  if ((abs(millis() - timer1)) > (int)(Intervalo_Amostragem)) { // Caso o tempo de amostragem tenha ocorrido, ele envia 1 para a função de filtro de media movel
    //Dessa forma a função sabe que é para atualizar o valor de saída para um novo valor filtrado
    filtro3v3(1);
  }
}

void imprimir(float valor1, float valor2, float valor3){ // A função recebe 3 parametros, e já imprime eles no formato correto.
  Serial.print(valor1);
  Serial.print(",");
  Serial.print(valor2);
  Serial.print(",");
  Serial.println(valor3);
}

float filtro3v3(bool atualiza_saida) { //O filtro aqui tem uma pequena mudanca em relacao ao filtro apresentado no post dedicado ao assunto
  //Esse filtro ele possui precisão variável. Ele começa bem impreciso e rápido, e vai mudando para impreciso e lento.
  //Isso porque não é esperado que o sinal de 3.3V mude com o tempo, por isso o filtro precisa começar bem rápido, para que atinja o valor esperado e vá corrigindo a precisão lá.
  static short Qtd_Amostras_Aux = 1;
  static float Saida_Filtro = 0;

  if (atualiza_saida == 0) return ((double)Saida_Filtro);

  else {
    Saida_Filtro += (float)(Leitura_3v3 - Saida_Filtro) / (float)Qtd_Amostras_Aux;
    Qtd_Amostras_Aux = constrain((Qtd_Amostras_Aux + 1), 0, Qtd_Amostras_3v3);
  }
}

float Tensao(){ // Essa funcao converte as mudanças no encoder e transforma em um valor de tensão.
  static float tensao = 0, aux; // Todas as variáveis são do tipo static porque o valor da tensão é salvo nessa função.
  static byte pos = 0; // A posição do digito que estamos digitando, correspondendo as casas decimais
  static bool button = false; // 
  if (encoderPos != 0) { // Encoder Pos é uma variável global que diz se o encoder foi para a direita ou esquerda. ele recebe -1 ou 1 se o encoder teve um acréscimo de movimentação.
   aux = tensao + encoderPos * pow(10, -(pos)); //A lógica é bem simples. Se o encoder teve um acrescimo, ele vai somar com a tensão que já existe 1x10^(-pos). Caso seja negativo, faz -1. Esse valor de 1 ou -1 são justamente os salvos na variável do encoder.
   // A variavel auxiliar é usada porque precisa passar por uma comparação para ver se a tensão selecionada é válida
    if ((aux >= -0.0001) and (aux <= (VccMax))) { // Esse IF é somente para evitar bugs de selecionar tensão menor que 0 e maior que a tensão máxima permitida.
      tensao = aux; // Se passou por esse IF, a tensão é válida e a tensão auxiliar passa a ser o valor da tensão final.
    }
    encoderPos = 0; // Faz com que a variável que sente a mudança do encoder retorne a 0, indicando que o encoder está parado agora.
  }
  ImprimeDisplay(tensao, pos); // Após isso simprime no display OLED o valor selecionado. A variavel de posição é chamada para colocar corretamente onde ficara a indicação de seleção.

  byte buttonState = digitalRead (buttonPin); // Aqui começa a verificação se o botão foi pressionado, para selecionar o algarismo correto.
  if (buttonState != oldButtonState) // Isso quer dizer que o botão foi pressionado
  {
    if (millis () - buttonPressTime >= debounceTime)      //Rotina para debounce
    {
      buttonPressTime = millis ();                        //Registra o tempo em que o botão foi pressionado
      oldButtonState =  buttonState;                      //Registra o antigo estado do botão
      if (buttonState == LOW)
      {
        pos = (pos + 1) % 3;  // Isso garante que a posição volta para o 0 depois do 2.
      }
    }
  }
  return ((float)tensao); // No final de tudo essa funcao retorna o valor da tensão que selecionamos.
}

void ImprimeDisplay(float tensao, byte pos) { // Funcao para imprimir no Display o valor da tensao junto com o traçado que indica qual o algarismo que está sendo mexido.
  display.setTextSize(2);                                   //Imprime as informações na tela size
  display.clearDisplay();                                   //Limpa o display

  display.setCursor(30, 10);                                //Seta o posição do texto
  display.print((float)abs(tensao));                        // Imprime a tensão no display
  display.println("V"); // E insere esse V no final para indicar Volts

  switch (pos) {     // Imprimindo o underline baseado em qual posição do algarismo escolhido está.
    case 0: display.setCursor(30, 15);
      break;

    case 1: display.setCursor(53, 15);
      break;

    case 2: display.setCursor(65, 15);
      break;
  }
  display.println("_");
  display.display(); //Atualiza o display
}

void PinA()                               //Rotina para tratar a interrupção causada pelo encoder rotativo PinA.
{
  cli();                                  //Pare as interrupções para que seja lido o valor nos pinos
  reading = PIND & 0xC;                   //Lê todos os oito valores dos pinos e remove todos, exceto os valores de pinA e pinB
  if (reading == B00001100 && aFlag)      //Checa se temos os dois os pinos em HIGH e se estavamos esperando uma borda ascendente deste pino
  {
    encoderPos = 1;                     //Volta para o 9 antes do 0
    bFlag = 0;                            //Reseta as flags para o próximo turno
    aFlag = 0;
  }
  else if (reading == B00000100)          //Sinaliza que estamos esperando o PinB realizar a transição para a rotação
    bFlag = 1;
  sei();                                  //Reinicia as interrupções
}

void PinB()                               //Rotina para tratar a interrupção causada pelo encoder rotativo PinB.
{
  cli();                                  //Pare as interrupções para que seja lido o valor nos pinos
  reading = PIND & 0xC;                   //Lê todos os oito valores dos pinos e remove todos, exceto os valores de pinA e pinB
  if (reading == B00001100 && bFlag)      //Checa se temos os dois os pinos em HIGH e se estavamos esperando uma borda ascendente deste pino
  {
    encoderPos = -1;
    bFlag = 0;                            //Reseta as flags para o próximo turno
    aFlag = 0;
  }
  else if (reading == B00001000)          //Sinaliza que estamos esperando o PinA realizar a transição para a rotação
    aFlag = 1;
  sei();                                   //Reinicia as interrupções
}