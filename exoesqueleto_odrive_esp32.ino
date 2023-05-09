// codigo ejemplo https://www.youtube.com/watch?v=pL3dhGtmcMY&t=1s

#include <WiFi.h>           // libreria conectar al wifi
#include <WebServer.h>      // crea servidor
#include "SuperMon.h"       // .h archivo donde se encuentra la interfaz
#include <ODriveArduino.h>  // libreria odrive


// variable donde se almacena para conectar a la wifi
#define USE_INTRANET

// pines del esp32 al que estan conectados el controlador odrive
#define ESP32_UART2_PIN_TX 17
#define ESP32_UART2_PIN_RX 16

// ODrive usa 115200 baud
#define BAUDRATE 115200

// Impresion con operador de flujo
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


// nombre wifi y contrasenya
#define LOCAL_SSID "duver"
#define LOCAL_PASS "b1324567"


// once  you are read to go live these settings are what you client will connect to
#define AP_SSID "duver"
#define AP_PASS "b1324567"


// start your defines for pins for sensors, outputs etc.
//#define PIN_OUTPUT 26 // connected to nothing but an example of a digital write from the web page
//#define PIN_FAN 27    // pin 27 and is a PWM signal to control a fan speed
#define PIN_LED 2     // 
#define PIN_A0 34     // pin sensor EMG
//#define PIN_A1 35     // some analog input sensor

// variables para almacenar datos y estados
float POS_REAL=0, POS_DESEADA=0;
float VoltsA0 = 0, VoltsA1 = 0;

bool LED0 = false, SomeOutput = false;
uint32_t SensorUpdate = 0;


bool ENCENDIDO = false;
bool APAGADO = false;
bool CALIBRADO = false;
bool SUBIR = false;
bool BAJAR =false;

float sensor_EMG;
//posicion del brazo en modo false esta abajo. modo true esta arriba
bool POSICION_BRAZO=false;

// declaracion de un array para mandar comunicarse con la interfaz
char XML[2048];

// declaracion de array de 32 elementos
char buf[32];

// variable para la ip cuando conecta a la intranet
IPAddress Actual_IP;

// las direcciones IP y la mascara de subred de la red a la que esta conectado el dispositivo
IPAddress PageIP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress ip;

// crea un servidor que escucha el puerto 80
WebServer server(80);



//crear un objeto ODriveArduino
ODriveArduino odrive(Serial1);

//declaracion de variables
float vel_limit = 22000.0f;
float current_lim = 11.0f;
float giro_deseado = 0.0f;


// imprime la ip para conectar la web
void printWifiStatus() {

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("Open http://");
  Serial.println(ip);
}




// boton para encender/apagar el motor
void ProcessButton_0() {

  LED0 = !LED0;
  digitalWrite(PIN_LED, LED0);
  Serial.print("Button 0 "); 
  Serial.println(LED0);

  // independientemente de si desea enviar cosas de vuelta al cliente o no
  // debes tener la linea de envio--ya que mantiene la pagina funcionando
  // mantener la pagina activa pero no enviar nada
  // aqui no necesito enviar un estado inmediato, cualquier estado
  // como el estado de iluminacion se enviara en la actualizacion de la pagina principal XML

  server.send(200, "text/plain", ""); //manda nada a la pagina web


}

//funcion para bajar el brazo modifica el valor de la variable subir a true
void ProcessButton_2() {

  BAJAR = true;
  server.send(200, "text/plain", ""); //manda nada a la pagina web

}

//funcion para subir el brazo modifica el valor de la variable subir a true
void ProcessButton_3() {

  SUBIR = true;
  server.send(200, "text/plain", ""); //manda nada a la pagina web

}


// funcion que enciende el motor
void FUNCION_CLOSED_LOOP_CONTROL(){

    if(ENCENDIDO==false){
       // print motor positions AXIS_STATE_CLOSED_LOOP_CONTROL
        int requested_state;
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        Serial << "Axis" << "c" << ": Requesting state " << requested_state << '\n';
        odrive.run_state(0, requested_state, false /*don't wait*/);     
        ENCENDIDO = true;
        APAGADO = false;
      Serial.print("APAGADO FALSE");
      }
}

// funcion para apagar el motor
void FUNCION_IDLE(){
  
    if(APAGADO==false){
      
      int requested_state;
      requested_state = AXIS_STATE_IDLE;
      Serial << "Axis" << 'i' << ": Requesting state " << requested_state << '\n';
      odrive.run_state(0, requested_state, false /*don't wait*/);

      ENCENDIDO = false;
      APAGADO = true;
      Serial.print("APAGADO TRUE");
      }
}

// funcion para calibrar el motor 0
void FUNCION_CALIBRADO(){

    if(CALIBRADO==true){
      int motornum = 0;
      int requested_state;

      requested_state = AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true)) return;

      requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

      requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
      CALIBRADO = false;
      }

}

//calcula la posicion deseada cuando el brazo sube
float calcular_posicion_deseada_s(){
  
  float pos_motor;
  float pos_motor_inicial;
  float pos_deseada;
  
    pos_motor = odrive.GetPosition(0);
    pos_motor_inicial = odrive.GetPosition(0);
    pos_deseada = pos_motor_inicial + 5.5;
    
    for ( pos_motor; pos_motor < 5.5f + pos_motor_inicial; pos_motor += 0.01f) {
  
       pos_motor;

    }
  giro_deseado = pos_motor;
  return giro_deseado;
  
}


// funcion para calcular la posicion a la que se desea llegar cuando baja
float calcular_posicion_deseada_a(){
  
  float pos_motor;
  float pos_motor_inicial;
  pos_motor = odrive.GetPosition(0);
  pos_motor_inicial = odrive.GetPosition(0);

  for ( pos_motor; pos_motor > pos_motor_inicial - 5.5f; pos_motor -= 0.01f) {

    pos_motor;
  }  
  giro_deseado = pos_motor;
  return giro_deseado;
}

// boton para calibrar el motor
void ProcessButton_1() {

  // just a simple way to toggle a LED on/off. Much better ways to do this
  Serial.println("Button 1 press");
  SomeOutput = !SomeOutput;

  //digitalWrite(PIN_OUTPUT, SomeOutput);
  Serial.print("Button 1 "); 
  Serial.println(LED0);
  //manda a la pagina web la informacion
  server.send(200, "text/plain", "");


}

// code to send the main web page
// PAGE_MAIN is a large char defined in SuperMon.h
void SendWebsite() {

  Serial.println("sending web page");
  // you may have to play with this value, big pages need more porcessing time, and hence
  // a longer timeout that 200 ms
  server.send(200, "text/html", PAGE_MAIN);

}


// funcion que manda el archivo paquete xml
void SendXML() {

  int input_pos_motor=0;
  
  strcpy(XML, "<?xml version = '1.0'?>\n<Data>\n");

  //la variable se multiplica por 100 ya que si no deja mandar decimales
  int entero_real = POS_REAL*100;
  sprintf(buf, "<B0>%d</B0>\n",entero_real);
  strcat(XML, buf);
  
  // send Volts0
  //sprintf(buf, "<V0>%d.%d</V0>\n", (int) (VoltsA0), abs((int) (VoltsA0 * 10)  - ((int) (VoltsA0) * 10)));
  //strcat(XML, buf);


  //la posicion deseada se multiplica por 100 para que no tenga decimales ya que da problemas
  int POS_DESEADA_2 = POS_DESEADA*100;
  sprintf(buf, "<B1>%d</B1>\n", POS_DESEADA_2);
  strcat(XML, buf);
  // send Volts1
  //sprintf(buf, "<V1>%d.%d</V1>\n", (int) (VoltsA1), abs((int) (VoltsA1 * 10)  - ((int) (VoltsA1) * 10)));
  //strcat(XML, buf);

  // show led0 status
  if (LED0) {
    strcat(XML, "<LED>1</LED>\n");
    FUNCION_CLOSED_LOOP_CONTROL();
    

  }
  else {
    strcat(XML, "<LED>0</LED>\n");
    FUNCION_IDLE();
  }

  if (SomeOutput) {
    strcat(XML, "<SWITCH>1</SWITCH>\n");
    FUNCION_CALIBRADO();

    
  }
  else {
    strcat(XML, "<SWITCH>0</SWITCH>\n");

    if(CALIBRADO==false){
      CALIBRADO = true;
      }
  }

  strcat(XML, "</Data>\n");
  // wanna see what the XML code looks like?
  // actually print it to the serial monitor and use some text editor to get the size
  // then pad and adjust char XML[2048]; above
  Serial.println(XML);

  // you may have to play with this value, big pages need more porcessing time, and hence
  // a longer timeout that 200 ms
  server.send(200, "text/xml", XML);

}


void setup() {
  
  // Serial to PC
  Serial.begin(BAUDRATE);
  
  //BAUDRATE: la velocidad de transmision en baudios de la comunicacion serie.
  //SERIAL_8N1: la configuracion de bits de datos, bits de paridad y bits de parada para la comunicacion serie.
  // En este caso, se utiliza 8 bits de datos, sin bits de paridad y 1 bit de parada.
  //ESP32_UART2_PIN_TX: el numero de pin GPIO que se utilizara como pin de transmision UART.
  //ESP32_UART2_PIN_RX: el numero de pin GPIO que se utilizara como pin de recepcion UART.
  Serial1.begin(BAUDRATE, SERIAL_8N1, ESP32_UART2_PIN_TX, ESP32_UART2_PIN_RX);

  // se espera a que monitor se abra
  while (!Serial) ;  
  Serial.println("Serial 0 Ready...");
  while (!Serial1) ; 
  Serial.println("Serial 1 Ready...");


  // se modifica la corriente y la velocidad
  for (int axis = 0; axis < 2; ++axis) {
    Serial1 << "w axis" << axis << ".controller.config.vel_limit " << vel_limit << '\n';
    Serial1 << "w axis" << axis << ".motor.config.current_lim " << current_lim << '\n';
  }

  // ya esta listo para comenzar
  Serial.println("Ready!");

  //el pin del led se pone como salida
  pinMode(PIN_LED, OUTPUT);

  // turn off led
  LED0 = false;
  digitalWrite(PIN_LED, LED0);

  //se declara el pin del sensor como entrada
  pinMode(PIN_A0, INPUT);


  /*El watchdog timer es un temporizador de hardware que esta disenyado para evitar que el dispositivo
   se bloquee o se cuelgue en caso de que ocurra un error en el software. Si el temporizador de vigilancia
   no se restablece periodicamente mediante una llamada al sistema, el dispositivo se reinicia automaticamente.*/
  disableCore0WDT();

  // maybe disable watch dog timer 1 if needed
  //  disableCore1WDT();

  // se incia el servidor
  Serial.println("starting server");

  // if you have this #define USE_INTRANET,  you will connect to your home intranet, again makes debugging easier
#ifdef USE_INTRANET
  WiFi.begin(LOCAL_SSID, LOCAL_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
  Actual_IP = WiFi.localIP();
#endif

  // if you don't have #define USE_INTRANET, here's where you will creat and access point
  // an intranet with no internet connection. But Clients can connect to your intranet and see
  // the web page you are about to serve up
#ifndef USE_INTRANET
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  WiFi.softAPConfig(PageIP, gateway, subnet);
  delay(100);
  Actual_IP = WiFi.softAPIP();
  Serial.print("IP address: "); Serial.println(Actual_IP);
#endif

  printWifiStatus();

  // Cuando el servidor web recibe una solicitud HTTP para la URL raiz, se invocara una funcion 
  // llamada "SendWebsite" que enviara la pagina web correspondiente al cliente.
  server.on("/", SendWebsite);


  //"Cuando el ESP recibe una cadena que contiene '/XML', construira y enviara un archivo XML.
  // De esta forma, podemos actualizar solo algunas partes de la pagina web."
  server.on("/xml", SendXML);


  //Si el usuario hace click en un boton de la interfaz, se enviara una 
  //solicitud HTTP al servidor web con una URL que coincida con la que se especifico en la llamada a "server.on()", 
  //y el manejador de eventos correspondiente se activara la funcion correspondiente.
  server.on("/BUTTON_0", ProcessButton_0);
  server.on("/BUTTON_1", ProcessButton_1);
  server.on("/BUTTON_2", ProcessButton_2);
  server.on("/BUTTON_3", ProcessButton_3);
  

  // inicia el servidor web
  server.begin();

}



void loop() {

  //se ejecuta cada 50ms 
  if ((millis() - SensorUpdate) >= 50) {
    //devuelve el numero de milisegundos transcurridos desde que la placa se encendio o se reinicio por ultima vez.
    SensorUpdate = millis();
    //se le lee el valor del pin A0 donde esta el sensor EMG
    sensor_EMG = analogRead(PIN_A0);

  
    if (BAJAR==true){
      // se cambia la variable de la posicion del brazo
      POSICION_BRAZO=false;
      //se guarda en la variable global la posicion deseada
      POS_DESEADA = calcular_posicion_deseada_a();
      //se cambia el valor de la variable 
      BAJAR = false;
      //declaracion de variables
      float pos_motor;
      float pos_motor_inicial;
      //se obtiene la posicion del motor
      pos_motor = odrive.GetPosition(0);
      //se obtiene la posicion del motor 
      pos_motor_inicial = odrive.GetPosition(0);
      
      // se hace un for para llevar el motor a la posicion deseada
      //se lleva desde la posicion inicial a la posicion inicial menos -5.5 que es la mediada para bajar el brazo
      //el aumento es de -0.01 hasta llegar a la posicion final
      for ( pos_motor; pos_motor > pos_motor_inicial - 5.5f; pos_motor -= 0.01f) {
        //se mueve el motor a la posicion deseada
        odrive.SetPosition(0, pos_motor);
        //se obtiene la posicion del motor
        POS_REAL = odrive.GetPosition(0);
        //se mantiene la pagina activa
        server.handleClient();
        delay(5);
      }
      }


    //si la variable subir esta en modo true entra en la condicion 
    //si la senyal que envia el sensor es superior a 2000 y la posicion de la ortesis estaba a bajo entoces entra en el if
    if (SUBIR==true || (sensor_EMG>2000 && POSICION_BRAZO==false)){

      //Se modifica la variable de posicion del brazo
      POSICION_BRAZO = true;
      //se guarda en la variable global la posicion a la cual tiene que ir el motor
      POS_DESEADA = calcular_posicion_deseada_s();
      Serial.println("Executing test move");

      //se modifica el valor de la variable 
      SUBIR = false;
      float pos_motor;
      float pos_motor_inicial;
      //se obtiene la posicion del motor
      pos_motor = odrive.GetPosition(0);
      //se obtiene la posicion incial del motor
      pos_motor_inicial = odrive.GetPosition(0);
      
      // se hace un for para llevar el motor a la posicion deseada
      //se lleva desde la posicion inicial a la posicion inicial mas 5.5 que es la mediada para subir el brazo
      //el aumento es de 0.01 hasta llegar a la posicion final
      for ( pos_motor; pos_motor < 5.5f + pos_motor_inicial; pos_motor += 0.01f) {

        //mover el motor a la nueva posicion
        odrive.SetPosition(0, pos_motor);
        //se lee la nueva posicion en la que esta
        POS_REAL = odrive.GetPosition(0);
        //se mantiene la pagina activa
        server.handleClient();
        delay(5);
      }
      }
  }

  //hay que llamar repetidamente a la funcion server.handleClient() de lo contrario la pagina no recibira instrucciones
  server.handleClient();

}
