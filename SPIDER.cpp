#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Endereço padrão do PCA9685
#define PCA9685_ADDR 0x40

// Frequência para servos
#define SERVO_FREQ 50  // 50 Hz

// Limites típicos de servo (ajusta depois!)
#define SERVO_MIN  150  // pulso mínimo
#define SERVO_MAX  600  // pulso máximo
#define posmin 40
#define posmax 140
#define zerope 45
#define doispe 55
#define quatrope 65
#define oitope 25
#define dezpe 50
#define dozepe 30

int pos;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// Forward declaration
void moveServo(uint8_t channel, uint8_t angle);
void Levanta_Aranha();
void Deita_Aranha();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Inicializando PCA9685...");

  // Iniciar I2C
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();  // GP4 = SDA, GP5 = SCL

  // Iniciar PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("PCA9685 pronto!");
  Serial.println("Movendo servo para posição central...");

  // Teste: mover servo do canal 0 para posição central
  pos=0;
  moveServo(0,90);        // CALIBRADA EM PÉ -> 45      Levantada para mover -> 
  moveServo(1,120);       // CALIBRADA EM PÉ -> 120     
  moveServo(2,55);        // CALIBRADA EM PÉ -> 55      Levantada para mover -> 
  moveServo(3,90);        // CALIBRADA EM PÉ -> 90
  moveServo(4,105);        // CALIBRADA EM PÉ -> 65      Levantada para mover -> 
  moveServo(5,65);        // CALIBRADA EM PÉ -> 65
  moveServo(6,90);
  moveServo(7,90);
  moveServo(8,25);        // CALIBRADA  EM PÉ -> 25     Levantada para mover ->
  moveServo(9,115);       // CALIBRADA EM PÉ -> 115
  moveServo(10,95);       // CALIBRADA EM PÉ -> 50      Levantada para mover ->
  moveServo(11,80);       // CALIBRADA EM PÉ -> 80
  moveServo(12,30);       // CALIBRADA EM PÉ -> 30      Levantada para mover ->
  moveServo(13,90);       // CALIBRADA EM PÉ -> 90
  moveServo(14,90); 
  moveServo(15,90);
  delay(3000);
}

void loop() {



         /* ROTINA 
  Deita_Aranha();
  delay(2000);
  
  Levanta_Aranha();
  delay(2000);
           */

}





// Função para mover servo por ângulo (0–180)
void moveServo(uint8_t channel, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);

  Serial.print("Servo ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.println(" graus");
}

void Levanta_Aranha(){
  for (pos=0; pos<=50; pos=pos+2){
    moveServo(0,zerope+50-pos);
    moveServo(2,doispe+50-pos);
    moveServo(4,quatrope+50-pos);
    moveServo(8,oitope+50-pos);
    moveServo(10,dezpe+50-pos);
    moveServo(12,dozepe+50-pos);
    delay(50);
  }
}

void Deita_Aranha(){
  for (pos=0; pos<=50; pos=pos+2){
    moveServo(0,zerope+pos);
    moveServo(2,doispe+pos);
    moveServo(4,quatrope+pos);
    moveServo(8,oitope+pos);
    moveServo(10,dezpe+pos);
    moveServo(12,dozepe+pos);
    delay(50);
  }
}
