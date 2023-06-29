#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <AltSoftSerial.h>

// pines definidos:
#define ReleMotor 5   // Rele(NC) que controla la alimentacion del CDI de la moto
#define ReleSistema 6 // Rele(NA) que controla la alimentacion de los modulos
#define PulsCte 2
AltSoftSerial BtModule;       //8 Rx, 9 Tx
SoftwareSerial GSMModule(3, 4);  // 3 Rx, 4 Tx
SoftwareSerial GPSModule(10, 11); // 10 Rx, 11 Tx
TinyGPSPlus gpsModule;

// Estados Posibles (MEF):
typedef enum
{
  EstadoDesactivado,
  EstadoEspera,
  LecturaBT,
  LecturaSMS
} estadoMEF;

estadoMEF EstadoActual;

// variables:
volatile unsigned int Milisegundos = 0, Segundos = 0;
char DatoBT = '0';

// Variables de SMS
String numTelefono;
String numeroPrincipal = "+5491135572199"; //"+5491167900710";
String numerosAux[5];
int numAux = 0;
String pw = "12345";
bool waitOp = false;
String msj;
bool ahorroFlag = false;

//////////////////////////////////////////////////// SET UP ////////////////////////////////////////////////////
void setup(){
  // Estado Pines:
  pinMode(ReleMotor, OUTPUT);
  pinMode(ReleSistema, OUTPUT);
  pinMode(PulsCte, INPUT_PULLUP);

  digitalWrite(ReleMotor, HIGH);
  digitalWrite(ReleSistema, LOW);

  // Interrupción externa:
  attachInterrupt(digitalPinToInterrupt(PulsCte), Interrumpido, RISING);

  // Timer setup:
  SREG = (SREG & 0b01111111);   // deshabilita la interrupción
  TIMSK2 = TIMSK2 | 0b00000001; // habilita inte. por desbordamiento
  TCCR2B = 0b00000011;          // prescaler de 32: 8M/32=250k -> T=4u; 256 x 4u = 1.025ms
  SREG = (SREG | 0b10000000);

  // Inicializa Serial(es)
  Serial.begin(9600);
  BtModule.begin(38400);
  GSMModule.begin(9600);
  GPSModule.begin(9600);

  delay(1000); // Espera que los modulos inicien correctamente

  // Configuracion de GSM
  GSMModule.println("AT+CMGF=1"); // Setea el GSM en modo texto
  delay(100);
  GSMModule.println("AT+CNMI=2,2,0,0,0"); // Envia los SMS al puerto serie
  delay(100);

  EstadoActual = EstadoDesactivado;
}

//////////////////////////////////////////////////// MAIN LOOP ////////////////////////////////////////////////////
void loop()
{
  switch (EstadoActual)
  {
  case EstadoDesactivado:
    digitalWrite(ReleMotor, HIGH);
    digitalWrite(ReleSistema, LOW);

    if (Segundos >= 2){
      Serial.println("Desactivado");

      if (BtModule.available()){
        DatoBT = BtModule.read();
        BtModule.write(DatoBT);
        BtModule.read();
        BtModule.read();
        EstadoActual = LecturaBT;
      }

      Segundos = 0;
    }
    break;

  case EstadoEspera:
    if (Segundos >= 2){
      Serial.println("Esperando");
      
      if (BtModule.available())
      {
        DatoBT = BtModule.read();
        Serial.println(DatoBT);
        BtModule.read();
        BtModule.read();
        EstadoActual = LecturaBT;
      }
      else if (GSMModule.available())
      {
        EstadoActual = LecturaSMS;
      }
      
      Segundos = 0;
    }
    break;

  case LecturaBT:
    if(DatoBT == '0'){EstadoActual = EstadoDesactivado; break;}

    else if(DatoBT == '1'){EstadoActual = EstadoEspera; ActivarSistema(); Serial.println("Sist.Activ.");}

    else if(DatoBT == '2'){EstadoActual = EstadoEspera; ActivarMotor(); Serial.println("Motor Activ.");}

    else if(DatoBT == '3'){EstadoActual = EstadoEspera; ModoAhorro(); Serial.println("Modo Ahorro");}

    //Config. main Num.
    else if(DatoBT == '4'){
      Segundos = 0; 
      EstadoActual = EstadoEspera;
      while(Segundos < 30){
        if (BtModule.available()) 
          numeroPrincipal = BtModule.readString();
          break;
      }
    }

    else if(DatoBT != '0' && DatoBT != '1' && DatoBT != '2' && DatoBT != '3' && DatoBT != '4'){
      Serial.println("Error");
      EstadoActual = EstadoDesactivado;
    }

  break;

  case LecturaSMS:
    msj = GSMModule.readString();
    if (msj.indexOf("+") != -1)
    {
      numTelefono = obtenerNumTelefono(msj); // Verifica si el SMS tiene un numero telefonico

      // Verifica si el numero principal
      if (numTelefono == numeroPrincipal)
      {
        enviarSMS(numTelefono, menuOptions());
        waitOp = true;
      }

      // Verifica si es algun numero auxiliar
      else if (verifNumSec(numTelefono))
      {
        enviarSMS(numTelefono, menuOptions());
        waitOp = true;
      }

      // Verifica si el SMS enviado es la contraseña de activacion
      else if (msj.indexOf(pw) != -1)
      {
        guardarNumAux(numTelefono);
        enviarSMS(numTelefono, "Número auxiliar guardado.");
        enviarSMS(numTelefono, menuOptions());
        waitOp = true;
      }

      // No cumplío ningun requisito
      else
      {
        enviarSMS(numTelefono, "No se ha podido registrar este número.");
      }
    }
    // Selecciona el modo
    else if (waitOp)
    {
      int op = msj.toInt();
      selectOp(numTelefono, op);
      waitOp = false;
    }
    break;

  default:
    EstadoActual = EstadoDesactivado;
    break;
  }
}

//////////////////////////////////////////////////// FUNCIONES ////////////////////////////////////////////////////

// Obtiene el numero de telefono del SMS
String obtenerNumTelefono(String msj)
{
  int inicio = msj.indexOf("+");
  int fin = msj.indexOf("/", inicio);
  String numTelefono = msj.substring(inicio, fin);
  return numTelefono;
}

// Recorre array de numeros secundarios
bool verifNumSec(String numTelefono)
{
  for (int i = 0; i < numAux; i++)
  {
    if (numTelefono == numerosAux[i])
    {
      return true;
    }
  }
  return false;
}

// Guarda numero auxiliar en el array de numeros secundarios
void guardarNumAux(String numTelefono)
{
  if (numAux < 5)
  {
    numerosAux[numAux] = numTelefono;
  }
  numAux++;
}

// Despliega el menú de opciones
String menuOptions()
{
  String menu = "Menú de opciones:\n\n";
  menu += "1. Activar/Desactivar Sistema\n";
  menu += "2. Activar/Desactivar Motor\n";
  menu += "3. Activar/Desactivar modo ahorro\n";
  menu += "4. Enviar Ubicación\n";
}

// Seleccionador (no se si esta bien dicho xd) de funciones
void selectOp(String numTelefono, int op)
{
  switch (op)
  {
  case 1:
    ActivarSistema();
    break;

  case 2:
    ActivarMotor();
    break;

  case 3:
    ModoAhorro();
    break;

  case 4:
    EnviarUbicacion();
    break;

  default:
    enviarSMS(numTelefono, "Opción Inválida");
    break;
  }
}

// Enviador(????) de SMS
void enviarSMS(String numTelefono, String msj)
{
  String trama = "AT+CMGS=\"+" + numTelefono + "\"\r";
  Serial.println("Enviando el Mensaje ...");
  GSMModule.print("AT+CMGF=1\r");
  delay(100);    //Espera que se cumplan los comandos
  GSMModule.print(trama);
  delay(500);
  GSMModule.print(msj);
  delay(500);
  GSMModule.print((char)26);  //Es requerido para el correcto funcionamiento
  delay(500);
  GSMModule.println();
  Serial.println("Texto Enviado.");
  delay(500); 
}

// Funcion para activar o desactivar el sistema
void ActivarSistema()
{
  if (EstadoActual == EstadoEspera && digitalRead(ReleSistema) == HIGH)
  {
    EstadoActual = EstadoDesactivado;
    digitalWrite(ReleSistema, LOW);
    enviarSMS(numTelefono, "Se ha desactivado el sistema.");
  }
  else if (EstadoActual == EstadoDesactivado && digitalRead(ReleSistema) == LOW)
  {
    EstadoActual = EstadoEspera;
    digitalWrite(ReleSistema, HIGH);
    enviarSMS(numTelefono, "Se ha activado el sistema.");
  }
}

// Funcion para activar o desactivar el motor
void ActivarMotor()
{
  if (digitalRead(ReleMotor) == HIGH)
  {
    digitalWrite(ReleMotor, LOW);
    enviarSMS(numTelefono, "Se ha desactivado el motor.");
  }
  else
  {
    digitalWrite(ReleMotor, HIGH);
    enviarSMS(numTelefono, "Se ha activado el sistema.");
  }
}

// Funcion para el envío del link de la ubicacion
void EnviarUbicacion()
{
  if (gpsModule.location.isValid() && gpsModule.location.isUpdated())
  {
    String ubicacion = obtenerLink(gpsModule.location.lat(), gpsModule.location.lng());
    enviarSMS(numTelefono, ubicacion);
  }
  else
  {
    enviarSMS(numTelefono, "No se ha podido obtener la ubicación.");
  }
}

// Funcion
void ModoAhorro()
{
  if (!ahorroFlag)
  {
    // Desactivar modo ahorro de energía
    GSMModule.println("AT+CFUN=1"); // Desactivar modo ahorro
    delay(100);
    enviarSMS(numTelefono, "Se ha desactivado el modo ahorro.");
    ahorroFlag = true;
  }
  else
  {
    // Activar modo ahorro de energía
    GSMModule.println("AT+CFUN=0"); // Activar modo ahorro
    delay(100);
    enviarSMS(numTelefono, "Se ha activado el modo ahorro.");
    ahorroFlag = false;
  }
}

String obtenerLink(float latitud, float longitud)
{
  String enlace = "https://maps.google.com/?q=" + String(latitud, 6) + "," + String(longitud, 6);
  return enlace;
}

  // función interrupción(desbordamiento):
ISR(TIMER2_OVF_vect)
{
  Milisegundos++;

  if (Milisegundos == 1000)
  {
    Segundos++;
    Milisegundos = 0;
  }
}
// función interrupción(externa):
void Interrumpido()
{
  EstadoActual = EstadoDesactivado;
  Serial.print("Desactiv forzosa");
}
