#include<Arduino.h>
#include<SoftwareSerial.h>
#include<AltSoftSerial.h>

//pines:
  SoftwareSerial GSMModule(3,4);// Rx, Tx
  AltSoftSerial BtModule; // 8Rx, 9Tx

  #define ReleMotor 5 // Rele(NC) que controla la alimentacion del CDI de la moto
  #define ReleSist 6  // Rele(NA) que controla laalimentación de los módulos
  #define PulsCte 2

//Estados MEF:
  typedef enum
  {
    Desactivado,
    Espera
  } estados;

  estados estActual;

//variables:
  volatile unsigned int Milisegundos = 0, Segundos = 0;
  char DatoBT = '0';

  //SMS
    String numTelefono;
    String numeroPrincipal = "5491135572199"; //"5491167900710"; "5491127115750";
    String numerosAux[5];
    int numAux = 0;
    String pw = "12345";
    bool waitOp = false;
    String msj;
    bool ahorroFlag = false;
    char datoSMS = '0';


void setup() {
  //Pines:
    pinMode(ReleMotor, OUTPUT);
    pinMode(ReleSist, OUTPUT);
    pinMode(PulsCte, OUTPUT);

    digitalWrite(ReleMotor, LOW);
    digitalWrite(ReleSist, HIGH);

  //Interrupción
    attachInterrupt(digitalPinToInterrupt(PulsCte), Emergencia, FALLING);

  //Timer
    SREG = (SREG & 0b01111111);   // deshabilita la interrupción
    TIMSK2 = TIMSK2 | 0b00000001; // habilita inte. por desbordamiento
    TCCR2B = 0b00000011;          // prescaler de 32: 8M/32=250k -> T=4u; 256 x 4u = 1.025ms
    SREG = (SREG | 0b10000000);

  //Seriales
    Serial.begin(9600);
    BtModule.begin(38400);
    GSMModule.begin(9600);

  //Config GSM
    GSMModule.println("AT+CMGF=1"); // Setea el GSM en modo texto
    delay(100);
    GSMModule.println("AT+CNMI=2,2,0,0,0"); // Envia los SMS al puerto serie
    delay(100);

  //Toqueteo GSM
    GSMModule.readString();
    GSMModule.flush();
    Serial.flush();

  estActual = Desactivado;
  Serial.println("Iniciado");

}

void loop() {
  switch(estActual)
  {
    case Desactivado:
      if(Segundos >= 2)
      {
        Serial.println("Desactivado");
        if(BtModule.available())
        {
          DatoBT = BtModule.read();
          BtModule.write("OK");
          LecturaBT();
        }
        Segundos = 0;
      }  
    break;

    case Espera:
      if(Segundos >=2)
      {
        Serial.println("Espera");
        if(GSMModule.available())
        {
          msj = GSMModule.readString();
          Serial.println(msj);
          LecturaSMS();
        }

        if(BtModule.available())
        {
          DatoBT = BtModule.read();
          Serial.println(DatoBT);
          LecturaBT();
        }
        Segundos = 0;
      }
    break;
  }
}

void enviarSMS(String telefono, String mensaje)
{
  String trama = "AT+CMGS=\"+" + telefono + "\"\r";
  Serial.println("Enviando");
  GSMModule.print("AT+CMGF=1\r");
  delay(100);
  GSMModule.print(trama);
  delay(500);
  GSMModule.print(mensaje);
  delay(500);
  GSMModule.print((char)26);
  delay(500);
  GSMModule.println();
  Serial.println("Enviado");
  delay(500);
}

//Función LecturaBT
void LecturaBT()
{
  BtModule.read();
  BtModule.read();
  Serial.println(DatoBT);
  switch(DatoBT)
  {
    case 0:
      BtModule.println(opcionesMenu());
    break;

    case 1:
      BtModule.println("Se apago el sistema")
      estActual = Desactivado;
    break;

    // case 2:
    //   estActual = Espera;
    // break;

    case 3:
      activMotor();
    break;

    case 4:
      ahorroEng();
    break;

    default:
      BtModule.println("Opcion Incorrecta");
    break;
}
}

//Función LecturaSMS
void LecturaSMS()
{
  if(msj.indexOf("Menu"))
  {
    enviarSMS(numeroPrincipal, opcionesMenu());    
  }

  else if(msj.indexOf(000))
  {
    enviarSMS(numeroPrincipal, "Se apago el sistema");
    estActual = Desactivado;
  }

  // else if(msj.indexOf(001))
  //   estActual = Espera;

  else if(msj.indexOf(002)){
    activMotor();
    if(digitalRead(ReleMotor) == HIGH)
    {
      enviarSMS(numeroPrincipal, "Se apago el motor");      
    }

    if(digitalRead(ReleMotor) == LOW)
    {
      enviarSMS(numeroPrincipal, "Se prendió el Motor");      
    }
  }

  else if(msj.indexOf(003))
  {
    ahorroEng();
    if(digitalRead(ReleSist) == HIGH)
    {
      enviarSMS(numeroPrincipal, "Módulos Encendidos");      
    }

    if(digitalRead(ReleMotor) == LOW)
    {
      enviarSMS(numeroPrincipal, "Módulos Apagados");          
    }
  }

  else
    enviarSMS(numeroPrincipal, "Opcion Invalida");
}

String opcionesMenu()
{
  String menu = "Menú de opciones:\n\n";
  menu += "1. Activar/Desactivar Sistema\n";
  menu += "2. Activar/Desactivar Motor\n";
  menu += "3. Activar/Desactivar modo ahorro\n";
  menu += "4. Enviar Ubicación\n";
}

//Emergencia
void Emergencia()
{
  estActual = Desactivado;
  Serial.println("Interrumpido");
}

//Función encender/apagar Motor
void activMotor()
{
  digitalWrite(ReleMotor, !digitalRead(ReleMotor));

  if(digitalRead(ReleMotor) == HIGH)
    Serial.println("Se apagó el Motor");
  
  if(digitalRead(ReleMotor) == LOW)
    Serial.println("Se prendió el Motor");
}

//Función ahorro energía
void ahorroEng()
{
  digitalWrite(ReleSist, !digitalRead(ReleSist));

  if(digitalRead(ReleSist) == HIGH)
    Serial.println("Módulos Encendidos");
  
  if(digitalRead(ReleMotor) == LOW)
    Serial.println("Módulos Apagados");
}

//Función Timer
ISR(TIMER2_OVF_vect)
{
  Milisegundos++;
  if (Milisegundos == 1000)
  {
    Segundos++;
    Milisegundos = 0;
  }
}
