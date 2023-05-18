#include <Arduino.h>
#include <SoftwareSerial.h>

//pines definidos:
#define ReleMotor 5       //Rele(NC) que controla la alimentacion del CDI de la moto
#define ReleSistema 6     //Rele(NA) que controla la alimentacion de los modulos
#define PulsCte 2
SoftwareSerial BtModule(10, 11); //10 Rx, 11 Tx
SoftwareSerial GSMModule(3, 4);

//Estados Posibles (MEF):
typedef enum{EstadoDesactivado, EstadoEspera, LecturaBT, LecturaSMS} estadoMEF;
estadoMEF EstadoActual;

//variables:
volatile unsigned int Milisegundos = 0, Segundos = 0;
char DatoBT;

void setup() {
  //Estado Pines:
  pinMode(ReleMotor, OUTPUT);
  pinMode(ReleSistema, OUTPUT);
  pinMode(PulsCte, INPUT);

  //Interrupción externa:
  attachInterrupt(digitalPinToInterrupt(PulsCte), Interrumpido, RISING);

  //Timer setup:
  SREG = (SREG & 0b01111111); //deshabilita la interrupción
  TIMSK2 = TIMSK2 | 0b00000001; //habilita inte. por desbordamiento
  TCCR2B = 0b00000011; //prescaler de 32: 8M/32=250k -> T=4u; 256 x 4u = 1.025ms
  SREG = (SREG | 0b10000000);

  //Inicializa Serial(es)
  BtModule.begin(38400);
  GSMModule.begin(9600);
}

void loop() {
  switch(EstadoActual) {
    case EstadoDesactivado:
      ActivarMotor();
      digitalWrite(ReleSistema, LOW);
      //'delay'
      if(BtModule.available()){
        DatoBT = BtModule.read();
        EstadoActual = LecturaBT;
      }
    break;

    case EstadoEspera:
    //'delay'
      if(BtModule.available()){
        DatoBT = BtModule.read();
        EstadoActual = LecturaBT;
      }

      else if(GSMModule.available())
        EstadoActual = LecturaSMS;
    break;

    case LecturaBT:
      switch(DatoBT){
        //
      }
    break;

    case LecturaSMS:
      //¿?
    break;

    default:
      EstadoActual = EstadoDesactivado;
    break; 
  }
}


//funciónes de acciones
void ActivarSistema(){
  EstadoActual = EstadoEspera;
  digitalWrite(ReleSistema, HIGH);
}
void DeactivarSistema(){
  EstadoActual = EstadoDesactivado;
  digitalWrite(ReleSistema, LOW);
}

void ActivarMotor(){
  digitalWrite(ReleMotor, LOW);
}
void DeactivarMotor(){
  digitalWrite(ReleMotor, HIGH);
}

void EnviarUbicación(){
  //no se cómo se hace xd
}

void ModoAhorro(){
  //
}


//función interrupción(desbordamiento):
ISR(TIMER_OVF_vect){
  Milisegundos++;

  if(Milisegundos = 1000){
    Segundos++;
    Milisegundos = 0;
  }
}
//función interrupción(externa):
void Interrumpido() {
  EstadoActual = EstadoDesactivado;
}