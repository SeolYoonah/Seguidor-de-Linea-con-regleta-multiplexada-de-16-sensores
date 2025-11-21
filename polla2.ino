// 1el motol
#define ENA 6
#define IN1 8
#define IN2 7

// 2do ñotor
#define ENB 5
#define IN3 9
#define IN4 4

typedef unsigned int uint;

const bool debug_mode = false  ;//lectura uwu balatro god 10/10

// Constantes que necesitamos calibrar, la parte más jodida
const float kP = 0.13920;     // Av 1.8 - 95-125 AV // 0.28920 - 60-160 AV // 21720 // 18920 130 AV
const float kD = 0.27820;     // Av 1.8 - 95-125 AV // 0.53820 - 60 - 160 AV // 49820 // 36820 130 AV
const float kI = 0.0009;  // 0.0017 - 60 - 160 AV
const int integral_max = 1000; // 
const uint integral_const = 20;

const uint base_speed = 50; // 
const uint max_speed = 130;
uint threshold = 650; // sensibilidad, a partir de qué punto se considera negro (solo hay que recalcar que entre más inclinado esté hacia abajo más sensiblidad se tendrá que poner para evitar ruido)
const uint diff = 1.4;

const uint adapt_delay = 250;
const uint calibration_tries = 10;

const uint calibration_button = 12;
uint calibration_state = 0;
bool vinibrado = false;

const uint offset = 7;

const uint analog_in = A4;
const uint sensors[] = { A0, A1, A2, A3 }; // meter cada sensor acá, hice el código dinámico (o intenté) así que si metemos más sensores no debería de haber problema, aunque lo testee solo con 6
const uint total_sensors = sizeof(sensors) / sizeof(sensors[0]);
const uint readers = 16; // escala en la cual se va a leer, en este caso son 16 sensores lolxd

const uint position_cap = (readers - 1) * 1000;
const uint set_point = (position_cap / 2);

int last_error = 0;
int integral = 0; 

void readSensors(uint* data) {
  for(int i = 0; i < readers; i++) {
    digitalWrite(sensors[0], (i & 0x01) ? HIGH : LOW);
    digitalWrite(sensors[1], (i & 0x02) ? HIGH : LOW);
    digitalWrite(sensors[2], (i & 0x04) ? HIGH : LOW);
    digitalWrite(sensors[3], (i & 0x08) ? HIGH : LOW);

   // delayMicroseconds(50);
    data[i] = analogRead(analog_in); //esto es para invertir la lectura porque me daba los valores al revés, si no funca solo hay que quitar el 1024
  }

  /* for(int i = 0; i < readers; i++) {
    Serial.print(data[i]);
    Serial.print(" | ");
  }*/
}

uint changeThreshold(uint* buffer, uint max) {
  uint maximun = max;
  
  for(int i = 0; i < readers; i++) {
    if(buffer[i] > maximun) {
      maximun = buffer[i];
    } 
  }

  return maximun;
}

void adaptThreshold() {
  uint buffer[readers];
  uint max = 0;

  for(int i = 0; i < calibration_tries; i++) {
    readSensors(buffer);
    max = changeThreshold(buffer, max);
  
    delay(adapt_delay);
  }

  threshold = max - (max / 6);

  delay(1000);

  vinibrado = true;
}

uint calculatePosition(uint* data) {
  uint sum_weighted_index = 0;
  uint sum_intensity = 0;
  uint active_sensors = 0;

  for (uint i = 0; i < readers; i++) {
      if (data[i] >= threshold) { 
        sum_intensity += data[i];
        sum_weighted_index += (uint)i * data[i]; 
        active_sensors++;
      }
  }

  if (sum_intensity == 0 || active_sensors == 0) {
      return (uint)-1; // Valor especial para indicar que se perdió la línea
  }

  uint position = (uint)((float)sum_weighted_index / sum_intensity * 1000.0);
    
  return position;
}

bool isSharpCurve(uint* data) {
  uint extreme_values = 0;
  for (uint i = 0; i < readers; i++) {
    if (data[i] >= threshold && (i == 0 || i == readers-1)) {
      extreme_values++;
    }
  }
  return extreme_values >= 2; // linea detectada en ambos extremos
}
 
void setMotor(uint leftSpeed, uint rightSpeed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, leftSpeed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);
}

void setup() {
  Serial.begin(9600);

  pinMode(calibration_button, INPUT);
  for(int i = 0; i < total_sensors; i++) pinMode(sensors[i], OUTPUT);
}

void loop() {
  calibration_state = digitalRead(calibration_button);

  if(calibration_state == LOW) {
    adaptThreshold(); 
  }

  if(!vinibrado) return;

  uint data[readers];
  readSensors(data);

  uint position = calculatePosition(data);
  int error;
  
  // esto es por ti heisenwolf 
  if (position == (uint)-1) {
    //
    error = last_error > 0 ? 1200: - 1200; // girar bruscamente en la última dirección conocida
  } else {
    error = position - set_point + offset;
  }

  integral += error;
  // Anti-windup
  if (integral > integral_max) integral = integral_max;
  if (integral < -integral_max) integral = -integral_max;
  
  int derivative = error - last_error;
  float proporcional = (kP * error);
  float derivativa = (kD * derivative);
  float integral_term = (kI * integral); 
  
  float correction = (proporcional) + (derivativa) + (integral_term);
  last_error = error;

  int motor_left_speed = base_speed + (int)correction;
  int motor_right_speed =  base_speed - (int)correction;

 
  if (isSharpCurve(data)) {
    motor_left_speed = constrain(motor_left_speed, 0, max_speed - integral_const);
    motor_right_speed = constrain(motor_right_speed, 0, max_speed - integral_const);
  } else {
    motor_left_speed = constrain(motor_left_speed, 0, max_speed);
    motor_right_speed = constrain(motor_right_speed, 0, max_speed);
  }

    

  if(debug_mode) {
    for(int i = 0; i < readers; i++) {
      Serial.print(data[i]);
      Serial.print(" | ");
    }

    Serial.print("Position: ");
    Serial.print(position);

    Serial.print(" | Error: ");
    Serial.print(error);

    Serial.print(" | T: ");
    Serial.print(threshold);

    Serial.print(" | L: ");
    Serial.print(motor_left_speed);

    Serial.print(" | R: ");
    Serial.print(motor_right_speed);

    Serial.print(" | C: ");
    Serial.print(correction);
    Serial.print(" | P: ");
    Serial.print (proporcional);
    Serial.print(" | d: ");
    Serial.print (derivativa);
    Serial.print(" | i: ");
    Serial.print (integral_term);

    Serial.println("");
    delay(500);

    return;
  }
  setMotor(motor_left_speed, motor_right_speed);

}