/* 
 * P A Q U I T O
 * ___________________________________________________________________
 *
 * Control del robot desde una Raspberry utilizando I2C.
 * @author blackzafiro
 *
 */
//#define DEBUG_PAQUITO
#define WITH_SERVOS

#include <Wire.h>
const int SLAVE_ADDRESS = 0x8; // Hexadecimal entre 8 y 127

// Voz
const int SPEAKER_PIN = 8;
const int LED_PIN = 13;

// Audio
const int MIC_PIN = A0;

// Sonar TODO
const int TRIG_PIN = 53;
const int ECHO_PIN = 52;

// Sonar base TODO
const int SONAR_SERVO = 2;

// Pan Tilt
const int PITCH_SERVO = 6;       // Arriba y abajo
const int ROLL_SERVO = 7;        // Ladeo
//const int YAW_SERVO = 5;

// Llantas
const int speedPinR = 9;            // Front Wheel PWM pin connect Model-Y M_B ENA 
const int RightMotorDirPin1 = 27;   // Front Right Motor direction pin 1 to Model-Y M_B IN1 (K1)
const int RightMotorDirPin2 = 26;   // Front Right Motor direction pin 2 to Model-Y M_B IN2 (K1)
const int RightMotorS1PinRA = 31;   // Front Right Motor encoder Signal 1 Orange
const int RightMotorS2PinRA = 30;   // Front Right Motor encoder Signal 2 Green

const int LeftMotorDirPin1 = 29;   // Front Left Motor direction pin 1 to Model-Y M_B IN3  (K3)
const int LeftMotorDirPin2 = 28;   // Front Left Motor direction pin 2 to Model-Y M_B IN4  (K3)
const int LeftMotorS1PinLA = 33;    // Front Left Motor encoder Signal 1 Orange
const int LeftMotorS2PinLA = 32;    // Front Left Motor encoder Signal 2 Green
const int speedPinL = 10;           // Front Wheel PWM pin connect Model-Y M_B ENB

const int speedPinRB = 11;          // Rear Wheel PWM pin connect Left Model-Y M_A ENA 
const int RightMotorDirPin1B = 23;  // Rear Right Motor direction pin 1 to Model-Y M_A IN1 (K1)
const int RightMotorDirPin2B = 22;  // Rear Right Motor direction pin 2 to Model-Y M_A IN2 (K1) 
const int RightMotorS1PinRB = 35;   // Rear Right Motor encoder Signal 1 Orange
const int RightMotorS2PinRB = 34;   // Rear Right Motor encoder Signal 2 Green

const int LeftMotorDirPin1B = 25;  // Rear Left Motor direction pin 1 to Model-Y M_A IN3  (K3)
const int LeftMotorDirPin2B = 24;  // Rear Left Motor direction pin 2 to Model-Y M_A IN4  (K3)
const int LeftMotorS1PinLB = 37;    // Rear Left Motor encoder Signal 1 Orange
const int LeftMotorS2PinLB = 36;    // Rear Left Motor encoder Signal 2 Green
const int speedPinLB = 12;          // Rear Wheel PWM pin connect Model-Y M_A ENB


#include <Speak.h>
#include <Car.h>

const Wheel WHEELS[] = {
  Wheel(speedPinL,  LeftMotorDirPin1,   LeftMotorDirPin2),
  Wheel(speedPinR,  RightMotorDirPin1,  RightMotorDirPin2),
  Wheel(speedPinLB, LeftMotorDirPin1B,  LeftMotorDirPin2B),
  Wheel(speedPinRB, RightMotorDirPin1B, RightMotorDirPin2B)
};

const Encoder ENCODERS[] = {
  Encoder(LeftMotorS1PinLA,  LeftMotorS2PinLA),
  Encoder(RightMotorS1PinRA, RightMotorS2PinRA),
  Encoder(LeftMotorS1PinLB,  LeftMotorS2PinLB),
  Encoder(RightMotorS1PinRB, RightMotorS2PinRB)
};

Car paquito(WHEELS, ENCODERS);

const int MAX_SPEED = 150;
const int MIN_SPEED = 50;

//
// Estado
//
int speed = 85;

bool speak = false;
bool sonar_around = false;


#include <Servo.h>
#ifdef WITH_SERVOS
Servo sonarBaseServo;
Servo pitchServo;
Servo rollServo;
//Servo yawServo;

int pitch_angle = 90;  // TODO: Make class
int roll_angle = 90;
int yaw_angle = 90;
#endif


//
// Inicializaciones
//
Voice voice(SPEAKER_PIN, LED_PIN);


// FL FR
// RL RR
enum Command {
  STOP =             0b00000000,
  BRAKE =            0b11001100,
  ACCELERATE =       0b00110011,
  FORWARD =          0b00001111,
  NE =               0b00001010, //right turn
  RIGHT =            0b01101001, //SE already exists
  SEAST =            0b10100000, //right back
  BACKWARD =         0b11110000,
  SW =               0b01000001, //left back
  LEFT =             0b10010110,
  NW =               0b00000101, //left turn
  CLOCKWISE =        0b01011010, //clockwise
  COUNTCLOCKWISE =   0b10100101, //countclockwise
  SPEAK =            0b00010001,
  SONAR_AROUND =     0b00100010, //spin sonar around
  PITCH_UP =         0b01000100, //look up
  PITCH_DOWN =       0b10001000, //look down
  ROLL_RIGHT =       0b01100110, //ladeo
  ROLL_LEFT =        0b10011001,
  YAW_RIGHT =        0b01110111, //turn right
  YAW_LEFT =         0b11101110,
};


void toCharArray(Command c, const char *commandName, int len) {
  String commandString;
  switch(c) {
    case STOP:
      commandString = "stop";
      break;
    case BRAKE:
      commandString = "brake";
      break;
    case ACCELERATE:
      commandString = "accelerate";
      break;
    case FORWARD:
      commandString = "forward";
      break;
    case NE:
      commandString = "NE";
      break;
    case RIGHT:
      commandString = "right";
      break;
    case SEAST:
      commandString = "SE";
      break;
    case BACKWARD:
      commandString = "backward";
      break;
    case SW:
      commandString = "SW";
      break;
    case LEFT:
      commandString = "left";
      break;
    case NW:
      commandString = "NW";
      break;
    case CLOCKWISE:
      commandString = "clockwise";
      break;
    case COUNTCLOCKWISE:
      commandString = "countclockwise";
      break;
    case SPEAK:
      commandString = "speak";
      break;
    case SONAR_AROUND:
      commandString = "sonar_around";
      break;
    case PITCH_UP:
      commandString = "pitch_up";
      break;
    case PITCH_DOWN:
      commandString = "pitch_down";
      break;
    case ROLL_RIGHT:
      commandString = "roll_right";
      break;
    case ROLL_LEFT:
      commandString = "roll_left";
      break;
    case YAW_RIGHT:
      commandString = "yaw_right";
      break;
    case YAW_LEFT:
      commandString = "yaw_left";
      break; 
    default:
      commandString = "unknown";
  }
  commandString.toCharArray(commandName, len);
}


void execute(Command c, unsigned char args[]) {
  switch(c) {
    case STOP:
#ifdef DEBUG_PAQUITO
      Serial.println("--> Detente");
#else
      paquito.stop();
#endif
      break;
    case BRAKE:
#ifdef DEBUG_PAQUITO
      Serial.println("--> brake");
#else
      if (speed > MIN_SPEED) {
        speed--;
      }
#endif
      break;
    case ACCELERATE:
#ifdef DEBUG_PAQUITO
      Serial.println("--> accelerate");
#else
      if (speed < MAX_SPEED) {
        speed++;
      }
#endif
      break;
    case FORWARD:
#ifdef DEBUG_PAQUITO
      Serial.println("--> Avanza " + args[0]);
#else
      paquito.moveForward(speed);
#endif
      break;
    case NE:
#ifdef DEBUG_PAQUITO
      Serial.println("--> NE");
#else
      paquito.moveNE(speed);
#endif
      break;
    case RIGHT:
#ifdef DEBUG_PAQUITO
      Serial.println("--> right");
#else
      paquito.moveRight(speed);
#endif
      break;
    case SEAST:
#ifdef DEBUG_PAQUITO
      Serial.println("--> SE");
#else
      paquito.moveSE(speed);
#endif
      break;
    case BACKWARD:
#ifdef DEBUG_PAQUITO
      Serial.println("--> Retrocede " + args[0]);
#else
      paquito.moveBackward(speed);
#endif
      break;
    case SW:
#ifdef DEBUG_PAQUITO
      Serial.println("--> SW");
#else
      paquito.moveSW(speed);
#endif
      break;
    case LEFT:
#ifdef DEBUG_PAQUITO
      Serial.println("--> left");
#else
      paquito.moveLeft(speed);
#endif
      break;
    case NW:
#ifdef DEBUG_PAQUITO
      Serial.println("--> NW");
#else
      paquito.moveNW(speed);
#endif
      break;
    case CLOCKWISE:
#ifdef DEBUG_PAQUITO
      Serial.println("--> clockwise");
#else
      paquito.rotateClockwise(speed);
#endif
      break;
    case COUNTCLOCKWISE:
#ifdef DEBUG_PAQUITO
      Serial.println("--> countclockwise");
#else
      paquito.rotateCounterClockwise(speed);
#endif
      break;
    case SPEAK:
      Serial.println("--> Dí algo " + args[0]);
      speak = true;
      break;
    case SONAR_AROUND:
      Serial.println("--> Sonar alrededor " + args[0]);
      sonar_around = true;
      break;
    default:
      Serial.print("--> Comando desconocido");
      Serial.println(c);
  }
  delay(100); // 0.1mm
}

long readDistance() {
  digitalWrite(TRIG_PIN,LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);             // Enviamos un pulso de 10us
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH) / 59;       // Distancia en cm
}

const int MIN_ANGLE = 80;
const int CENTER_ANGLE = 90;
const int MAX_ANGLE = 100;
void sense_sonar_around() {
  int pos = 0;
  
  Serial2.print("[DIST] ");
  for (pos = CENTER_ANGLE; pos <= MAX_ANGLE; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    sonarBaseServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    Serial2.print(readDistance());
    Serial2.print(" ");
  }
  for (pos = MAX_ANGLE; pos >= MIN_ANGLE; pos -= 1) { // goes from 180 degrees to 0 degrees
    sonarBaseServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    Serial2.print(readDistance());
    Serial2.print(" ");
  }
  for (pos = MIN_ANGLE; pos <= CENTER_ANGLE; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    sonarBaseServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    Serial2.print(readDistance());
    Serial2.print(" ");
  }
  Serial2.println("[/DIST]");
}


void setup() {
  // Depurado usando un puerto serial
  Serial.begin(9600);
  Serial2.begin(9600);

  // Se sube al camión I2C como esclavo con la dirección indicada
  Wire.begin(SLAVE_ADDRESS);

  // Voz
  randomSeed(analogRead(0));
  voice.begin();

  // Audio
  pinMode(MIC_PIN, INPUT);

  // Sonar
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Servos
#ifdef WITH_SERVOS
  sonarBaseServo.attach(SONAR_SERVO);
  sonarBaseServo.write(CENTER_ANGLE);
  
  pitchServo.attach(PITCH_SERVO);
  rollServo.attach(ROLL_SERVO);
  
  pitchServo.write(90);
  rollServo.write(90);
#endif  

  // Inicializa los pins del carrito
  paquito.begin();
  Serial.println("*.´`. paquito init .´`.*");
#ifdef DEBUG_PAQUITO
  Serial.println("---DEBUG MODE---> NO MOTORS");
#endif

  voice.babble();

  //paquito.testWheels();
  //paquito.moveForward(100);
  //delay(1000);
  //paquito.stop();

  // Se ejecuta al recibir datos
  Wire.onReceive(receiveEvent);

  // Llama requestEvent cuando se soliciten datos
  Wire.onRequest(requestEvent);
}

// Función que se ejecuta cuando el maestro va a enviar información.
void receiveEvent(int howMany) {
  char buf[200];
  sprintf(buf, "\n...~~~ Received %d:\n", howMany);
  Serial.print(buf);

  char ini = Wire.read();
  sprintf(buf, "First byte: %d\n", ini);
  Serial.print(buf);

  if (howMany < 2) {
    Serial.println("~~~\n");
    return;
  }
  unsigned char commandByte = Wire.read();

  const int commandBufLength = 15;
  char commandName[commandBufLength];
  toCharArray(commandByte, commandName, commandBufLength);
  sprintf(buf, "Command: %d %s\n", commandByte, commandName);

  Serial.print(buf);
  

  Serial.println("Arguments:");
  unsigned char args[howMany - 2];
  int ind = 0;
  int ava;
  while (ava = Wire.available()) {
    unsigned char c = Wire.read(); // receive byte as a character
    sprintf(buf, "Wire read %d: %d=%c\n", ava, c, c);
    Serial.print(buf);

    args[ind++] = c;
  }

  execute(commandByte, args);
  Serial.println("~~~...\n");
}

// Función que se ejecuta cuando el maestro solicita información.
void requestEvent() {

}

void loop() {
  //delay(100); // 0.1mm
  // Sonar
  long distance;
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);             // Enviamos un pulso de 10us
  digitalWrite(TRIG_PIN, LOW);
  distance = pulseIn(ECHO_PIN, HIGH) / 59;       // Distancia en cm
  if (distance < 10) {
    paquito.stop();
  }

  // Enviar información del codificador
  Serial2.print("[ENC] ");
  for(int i = 0; i < Car::NUM_WHEELS; i++)
  {
    Serial2.print(paquito.count(i));
    Serial2.print(" ");
  }
  Serial2.println("[/ENC]");

  // Enviar información del micrófono
  Serial2.print("[MIC] ");
  Serial2.print(analogRead(MIC_PIN));
  Serial2.println("[/MIC]");

  if (speak) {
    voice.babble();
    speak = false;
  }
  if (sonar_around) {
    sense_sonar_around();
    sonar_around = false;
  }
}
