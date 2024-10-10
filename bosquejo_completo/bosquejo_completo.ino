/*
 * Taller de Robótica Proyecto PAPIME PE104223
 * @author blackzafiro
 *
 */

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
const int PITCH = 6;       // Arriba y abajo
const int ROLL = 7;        // Ladeo

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

// Estado de movimiento
//bool mueve[Car::NUM_WHEELS] = {false, false, false, false};
int speed = 70;

// Odómetro
//nt readings[2];
//int counter;

//#include <Wire.h>
//const int SLAVE_ADDRESS = 0x8; // Hexadecimal entre 8 y 127


// Setups
Voice voice(SPEAKER_PIN, LED_PIN);

#define SETUP_SPEAK() {\
  randomSeed(analogRead(0));  \
  voice.begin(); \
}\

/*
const int SETUP_AUDIO() {\
  pinMode(MICRO_PIN, INPUT);\
}\
*/

#define SETUP_CAR() {\
  paquito.begin(); \
  Serial.println("*.´`. paquito init .´`.*"); \
}\

void simpleCommand(int code) {
  switch(code) {
    case '5':
      paquito.stop();
      //for(int i = 0; i < Car::NUM_WHEELS; i++) mueve[i] = false;
      break;
    case '8':
      paquito.moveForward(speed);
      //for(int i = 0; i < Car::NUM_WHEELS; i++) mueve[i] = true;
      break;
    case '2':
      paquito.moveBackward(speed);
      //for(int i = 0; i < Car::NUM_WHEELS; i++) mueve[i] = true;
      break;
    /*case 'i':
      switchWheel(FL);
      break;
    case 'o':
      switchWheel(FR);
      break;
    case 'k':
      switchWheel(BL);
      break;
    case 'l':
      switchWheel(BR);
      break;*/
    case 's':
      voice.randomPhrase();
      break;
  }
}
/*
void updateEncoder() {

  for(int i = 0; i < Car::NUM_WHEELS; i++) {
    /*
    ENCODERS[i].read(readings);
    Serial.print(readings[0]);
    Serial.print(" ");
    Serial.print(readings[1]);
    Serial.print(" "); */
/*    counter = ENCODERS[i].count();
    Serial.print(counter);
    Serial.print(" ");
  }
  Serial.println();
  delay(50);
}
*/
/*
void switchWheel(WheelId id) {
  if (mueve[id]) {
    paquito.stop(id);
    mueve[id] = false;
  } else {
    //aLastState = digitalRead(RightMotorS1PinRB);
    paquito.moveForward(id, speed);
    mueve[id] = true;
  }
}
*/

/*
 * WiFi
 */

//#include <inttypes.h>
#include "WiFiEsp.h"
//include "WiFiEspClient.h"
//#include "WiFiEspServer.h"

char ssid[] = "paquito_zero"; 
int status = WL_IDLE_STATUS;
char packetBuffer[5];           // ring buffer
unsigned int localPort = 80;    // local port to listen on

WiFiEspServer server(localPort);

void SETUP_WIFI() {
  Serial1.begin(115200);
  Serial1.write("AT+UART_DEF=9600,8,1,0,0\r\n");
  delay(200);
  Serial1.write("AT+RST\r\n");
  delay(200);
  Serial1.begin(9600);    // initialize serial for ESP module
  WiFi.init(&Serial1);    // initialize ESP module
  
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("No se detectó el escudo WiFi");
    // aquí se queda
    while (true);
  }
  
  Serial.print("Intentando inicializar AP ");
  Serial.println(ssid);
  //AP mode
  status = WiFi.beginAP(ssid, 10, "", 0);
  /*if (status != WL_AP_LISTENING) {
    Serial.println("Falló la creación del punto de acceso.");
    // aquí se queda
    while (true);
  }*/
  
  Serial.println("Estás conectado a la red");
  printWifiStatus();
  //Udp.begin(localPort);
  server.begin();
  
  Serial.print("Escuchando en el puerto ");
  Serial.println(localPort);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("Dirección IP: ");
  Serial.println(ip);

  // print where to go in the browser
  Serial.println();
  Serial.print("Servidor en ");
  Serial.println(ip);
  Serial.println();
}

void verifyWiFiStatus() {
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    /*if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Dispositivo conectado a PA");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("El dispositivo se desconectó del PA");
    }*/
    Serial.print("Estado de AP ");
    Serial.println(status);
  }
}

/*void serveClient(Client client) {
  Serial.println("Llegó un cliente nuevo"); // print a message out the serial port
}*/
/*
void serveClient(WiFiEspClient client) {
  Serial.println("Llegó un cliente nuevo"); // print a message out the serial port
  String currentLine = "";                  // make a String to hold incoming data from the client
  while (client.connected()) {              // loop while the client's connected
    delayMicroseconds(10);                  // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
    int numBytes = client.available();
    //Serial.write("Bytes recibidos: ");
    //Serial.println(numBytes);
    //uint8_t buf[numBytes];
    if (numBytes) {                         // if there's bytes to read from the client,
      char c = client.read();               // read a byte, then
      //Serial.write(c);                    // print it out to the serial monitor
      if (c == '\n') {                      // if the byte is a newline character

        // if the current line is blank, you got two newline characters in a row.
        // that's the end of the client HTTP request, so send a response:
        if (currentLine.length() == 0) {
          Serial.println("**** Enviando respuesta y terminando. ****");
          // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
          // and a content-type so the client knows what's coming, then a blank line:
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
/*
          // the content of the HTTP response follows the header:
          client.print("<p style=\"font-size:7vw;\">Click <a href=\"/H\">here</a> turn the LED on<br></p>");
          client.print("<p style=\"font-size:7vw;\">Click <a href=\"/L\">here</a> turn the LED off<br></p>");
*/
          // The HTTP response ends with another blank line:
/*          client.println();
          // break out of the while loop:
          break;
        }
        else {      // if you got a newline, then clear currentLine:
          Serial.print("Line: ");
          Serial.println(currentLine);
          currentLine = "";
        }
      }
      else if (c != '\r') {    // if you got anything else but a carriage return character,
        currentLine += c;      // add it to the end of the currentLine
      }

      /*
      // Check to see if the client request was "GET /H" or "GET /L":
      if (currentLine.endsWith("GET /H")) {
        digitalWrite(led, HIGH);               // GET /H turns the LED on
      }
      if (currentLine.endsWith("GET /L")) {
        digitalWrite(led, LOW);                // GET /L turns the LED off
      }
      */      
/*    }
  }
}
*/

// https://github.com/bportaluri/WiFiEsp/blob/master/examples/WebServerAP/WebServerAP.ino
//void serveEspClient(bool client) {
//void serveEspClient(WiFiEspClient client) {
//}

// BOARD WORK

void setup() {
  Serial.begin(9600);
  SETUP_WIFI();
  SETUP_SPEAK()
  //SETUP_AUDIO()
  SETUP_CAR()

  voice.randomPhrase();
  //listen(10);
}

void loop() {
  /*if (mueve[0] || mueve[1] || mueve[2] || mueve[3]) {
    updateEncoder();
  }*/

  verifyWiFiStatus();

  WiFiEspClient client = server.available();   // listen for incoming clients
  if (client) {                                // if you get a client,
    //serveClient(client);
    //serveEspClient(client);
    //serveEspClient(client);
    // cierra la conexión:
    client.stop();
    Serial.println("El cliente se desconectó\n");
  }

}

void serialEvent() {
  if(Serial.available() > 0 ){
    char datum = Serial.read();
    Serial.println(datum);
    simpleCommand(datum);
  }
}
