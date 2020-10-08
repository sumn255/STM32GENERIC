#include <mecarun_v2.h>

Mecarun_v2 mycar;

void setup() {
  // put your setup code here, to run once:
  //int16_t speed[]={-800,-800,-800,-800};
  int16_t xyr[]={-100,0,0};
  pinMode(PD2,OUTPUT);
  digitalWrite(PD2,HIGH);
  delay(50);
  digitalWrite(PD2,LOW);
  delay(50);
  //mycar.Set_speed_bypass(speed);
  mycar.PID_Enable(1.04500008, 0.0900062919, 0);
  mycar.Move(xyr);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*digitalWrite(PD2,HIGH);
  delay(50);
  digitalWrite(PD2,LOW);
  delay(50);*/
  mycar.Set_speed();
}
