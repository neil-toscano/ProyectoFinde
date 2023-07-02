#include <Arduino.h>
#include <ArduinoJson.h>
#include <time.h>
#include <NTPClient.h>
#include <FirebaseESP32.h>
#include <WiFiUdp.h>
#include <DHTesp.h>


#define LED_PIN     14
#define NUM_LEDS    10
#define CLOCK_PIN 18  // Pin de reloj para el bus SPI
#define DATA_PIN 23  // Pin de datos para el bus SPI



struct Persona {
  int edad;
  
};

FirebaseJson json;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "south-america.pool.ntp.org");

const char* ssid = "moto g(20)";
const char* password = "123456789";


//Decaramos el variable que almacena el pin a conectar el DHT22
#define pinDHT        4
#define powerDHT      2
#define EN            33
#define InB           25
#define InA           32
#define Heating       14
#define Humidifier    5

#define PARAMETROS_CONTROL 4

#define FIREBASE_HOST "https://projectocarrera-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "AIzaSyCFj27zofYoHq-vCgG5jasliB0OZHG-mfw"


//------------------------------FIREBASE-----------------------------------
FirebaseData firebaseData;
FirebaseJsonData result;



float TempRangeMax = 20;//Rango máximo de temperatura
float TempRangeMin = 12;//Rango minimo de temperatura
float HumRangeMax = 85; //Rango maximo de humedad de aire
float HumRangeMin = 70; //Rango maximo de humedad de aire

struct ParametrosMedicion
{
  float temperatura;
  float humedad;
  int humedadsuelo;
  int luz;
};

ParametrosMedicion MedicionAnterior = {0.0,0.0,0,0};


// ARRAY QUE RECOGERA LAS VARIABLES DE MANUAL U AUTOMATICO DE LOS PARAMETROS DE RIEGO
byte ButtonControl[PARAMETROS_CONTROL]={0,0,0,0};
//ButtonControl[0]: Temperatura
  //Si ButtonControl[0] = 0, el modo automático esta activado
  //Si ButtonControl[0] = 1, el modo manual esta activado

//ButtonControl[1]: Humedad de aire
  //Si ButtonControl[1] = 0, el modo automático esta activado
  //Si ButtonControl[1] = 1, el modo manual esta activado

//ButtonControl[2]: Humedad de suelo
  //Si ButtonControl[2] = 0, el modo automático esta activado
  //Si ButtonControl[2] = 1, el modo manual esta activado

//ButtonControl[3]: Luz
  //Si ButtonControl[3] = 0, el modo automático esta activado
  //Si ButtonControl[3] = 1, el modo manual esta activado    

//A su vez, cada estado tiene un número asociado para el switchcase del loop()
//Esto es por la fórmula 2*i+ButtonControl[i]

//Los valores asociados a cada estado resultan:
  //Si ButtonControl[0] = 0  -------> 0 case ----> Temperatura en modo automatico
  //Si ButtonControl[0] = 1  -------> 1 case ----> Temperatura en modo manual
  //Si ButtonControl[1] = 0  -------> 2 case ----> Humedad del aire en modo automatico
  //Si ButtonControl[1] = 1  -------> 3 case ----> Humedad del aire en modo manual
  //Si ButtonControl[2] = 0  -------> 4 case ----> Humedad del suelo en modo automatico
  //Si ButtonControl[2] = 1  -------> 5 case ----> Humedad del suelo en modo manual
  //Si ButtonControl[3] = 0  -------> 6 case ----> Luz en modo automatico
  //Si ButtonControl[3] = 1  -------> 7 case ----> Luz en modo manual


bool EncenderOApagar = false; //boolean de control

//Accionador de los ventiladores y calefaccion
void AccionarVentilacionYCalor(bool ventilacion, bool calentamiento);
//Accion de control de la humedad
void encenderHumidificador();
void apagarHumidificador();
void GestionActuadoresHumedad(float hum);
void GestionActuadoresTemperatura(float temper);
void enviarDatosFirebase(float temperatura);
void conectarWiFi();
void inicializarFirebase();
//Instanciamos el DHT
DHTesp dht;
//int aux =0;
void setup() {
//------------------------------FIREBASE-----------------------------------
  Serial.begin(115200);
  
 // conectarWiFi();//conexion 
  //timeClient.setTimeOffset(-18000);// la hora esta sincronizado con un servidor de Estados unidos
  //Existe un desface de 5 horas
 // timeClient.begin();// iniciamos la conexion
  
 // Serial.println("Conexión exitosa");
  
 // inicializarFirebase();
  
  
  //Alimentacion del DHT22
 // pinMode(powerDHT,OUTPUT);
 // digitalWrite(powerDHT,HIGH);
  //Inicializamos el dht22
  //dht.setup(pinDHT, DHTesp::DHT22);


  pinMode(EN, OUTPUT);
  pinMode(InA, OUTPUT);
  pinMode(InB, OUTPUT);
  pinMode(Heating, OUTPUT);
  digitalWrite(Heating,HIGH);
  pinMode(Humidifier,OUTPUT); 
// leer();

 
}
void loop() {
  

 // Establecer un color rojo en el primer LED
  
  //leer();
  //delay(1000);
}

void GestionActuadoresTemperatura(float temper){
  if (temper > TempRangeMax){
    AccionarVentilacionYCalor(true, false);
  }
  else if (temper < TempRangeMin)
  {
    AccionarVentilacionYCalor(true, true);
  }
  else{
    AccionarVentilacionYCalor(false, false);
  }
}

void GestionActuadoresHumedad(float hum){
  if(hum < HumRangeMin){
    encenderHumidificador();
  }
  else{
    apagarHumidificador();
  }
}


void AccionarVentilacionYCalor(bool ventilacion, bool calentamiento){
  digitalWrite(EN,ventilacion);
  digitalWrite(InA, LOW);
  digitalWrite(InB, ventilacion);
  digitalWrite(Heating, ~calentamiento);
}

void encenderHumidificador(){
  digitalWrite(Humidifier,HIGH);
  delay(500);
  digitalWrite(Humidifier, LOW);

}

void apagarHumidificador(){
  digitalWrite(Humidifier,HIGH);
  delay(500);
  digitalWrite(Humidifier, LOW);
  delay(1000);
  digitalWrite(Humidifier,HIGH);
  delay(500);
  digitalWrite(Humidifier, LOW);
}



//------------------------------FIREBASE-----------------------------------
void conectarWiFi() {///WIFI====================
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.print("Conexión WiFi establecida. Dirección IP: ");
  Serial.println(WiFi.localIP());
}
void inicializarFirebase() {
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Firebase.setMaxRetry(firebaseData, 3);
}

void enviarDatosFirebase(float temperatura) {
   timeClient.update();
// Obtiene el tiempo actual en segundos desde el 1 de enero de 1970
  time_t currentTime =  timeClient.getEpochTime(); // Obtener el tiempo en formato epoch

  // Convierte el tiempo actual a una estructura de tiempo local
  struct tm *localTime = localtime(&currentTime);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", localTime);

  // Imprimir la hora formateada
  Serial.println(buffer);
  

  json.set("valor", temperatura);
  json.set("fecha",buffer);
  json.set("nombre","Valor_temp");
  if (Firebase.pushJSON(firebaseData, "sensores/temperatura", json)) {
    Serial.println("Datos enviados correctamente");
  } else {
    Serial.println("Error al enviar los datos");                 
    Serial.println(firebaseData.errorReason());
  }
  
}
void leerDatos() {
  Serial.println("Error al leer el valor");
  if (Firebase.getString(firebaseData, "/sensores/estado_vent")) {
    // Crea un objeto FirebaseJsonData para almacenar el resultado
    FirebaseJsonData jsonData;

    // Lee el valor de la propiedad específica y almacénalo en jsonData
    if (json.get(jsonData, "Apagar")) {
      // Verifica si el valor es válido
      if (jsonData.success) {
        // Imprime el valor leído
        Serial.println("Valor: " + String(jsonData.intValue));
      } else {
        Serial.println("Error al leer el valor");
      }
    } else {
      Serial.println("Error al leer los datos");
    }
  } else {
    Serial.println("Error al obtener los datos");
  }
}
void leer() {
  
    if (Firebase.getJSON(firebaseData, "/sensores/luz/")) {
     FirebaseJson &json = firebaseData.to<FirebaseJson>();
     json.get(result , "B");
     int ga=result.to<int>();
     if(ga<5){
 Serial.println("hola");
     }
     else{
      Serial.println(ga);
     }
    

  } else {
    Serial.println(firebaseData.errorReason());
  }
}
