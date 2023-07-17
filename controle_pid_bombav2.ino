#include <HX711.h>

// Configuração da célula de carga HX711
const int LOADCELL_DOUT_PIN = A1;
const int LOADCELL_SCK_PIN = A0;
HX711 scale;

float weight;
// Configuração da ponte H L298
const int ENA_PIN = 5;    // Pino de controle de velocidade da bomba de água
const int IN1_PIN = 6;    // Pino de controle da direção da bomba de água
const int IN2_PIN = 7;    // Pino de controle da direção da bomba de água

// Constantes do controlador PID
const float Kp = 2.7-0.3;    // Ganho proporcional
const float Ki = 0.006/60;    // Ganho integral
const float Kd = 0;    // Ganho derivativo

float P, I, D;


// Variáveis do controlador PID
float setpoint = 300.0;  // Nível de referência em gramas
float input, output;
float error, lastError;
float integral, derivative;

// Variáveis para o cálculo do tempo
unsigned long lastTime;
unsigned long sampleTime = 100;  // Tempo de amostragem em milissegundos

void setup() {
  // Inicialização dos pinos
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Configuração da célula de carga
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(476.45);

  // Tempo de estabilização da célula de carga
  delay(2000);

  // Inicialização da comunicação serial
  Serial.begin(115200);

  // Inicialização do controlador PID
  lastTime = millis();
  lastError = 0;
  integral = 0;
}

void loop() {
  // Cálculo do tempo decorrido desde a última amostra
  unsigned long now = millis();
  unsigned long deltaTime = now - lastTime;

    // Calcula o erro
    error = setpoint - weight;

    
  // Verifica se é hora de realizar uma nova amostra
  if (deltaTime >= sampleTime) {
    // Realiza a leitura do peso
    weight = scale.get_units()*(-1) - 800;


    // Calcula as componentes do controlador PID
    integral += (error * deltaTime);
    
    derivative = (error - lastError) / deltaTime;

    P = Kp*error;

    if(P<0){
      P = 0;
    }else if(P>255){
      P = 255;      
    }


    I = Ki*integral;

    if(I<0){
      I = 0;
    }else if(I>255){
      I = 255;      
    }


    D = Kd*derivative;

    if(D<0){
      D = 0;
    }else if(D>255){
      D = 255;      
    }

    // Calcula a saída do controlador PID
    output = (P+I+D);     //fator de ganho


     //map(output, 0, 636592.43, 255, 110); 
    // Limita a saída entre 0 e 255 (para usar com a ponte H)
    
    
    if (output < 0) {
      output = 0;
    } else if (output > 255) {
      output = 255;
    }
    

 // Atualiza a saída da bomba e imprime os valores no Monitor Serial
    analogWrite(ENA_PIN, output);
    lastTime = now;

    // Atualiza o estado dos pinos da ponte H para controlar a direção da bomba de água
      digitalWrite(IN1_PIN, HIGH);
      digitalWrite(IN2_PIN, LOW);


    Serial.print("Erro: ");
    Serial.print(error );
    Serial.print(" ");

    Serial.print(" Integrador ");
    Serial .print(I);
    Serial.print(" ");
    
    Serial.print(" Proporcional ");
    Serial .print(P);
    Serial.print(" ");
    

    // Imprime os valores no Monitor Serial
    Serial.print(" ");
    Serial.print(weight);
    Serial.print(" g | Saída: ");
    Serial.println(output);

    // Atualiza as variáveis para a próxima iteração
    lastError = error;
  }
}
