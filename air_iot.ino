#include "TonyS_X1.h"

bool air1_mode_auto=true, air2_mode_auto=true;

void setup() 
{
  Serial.begin(115200);
  Serial.println("TonyS_X1 Example");
  
  Tony.begin();  //----  begin Library
  delay(10);
  Tony.pinMode(LED_BUILTIN, OUTPUT); //----  Set Pin IO12 (LED_BUILTIN) to OUTPUT
}

void loop() 
{
  /// Air 1
   air1_auto_mode = Tony.digitalRead(DI1);
   air1_manual_mode = Tony.digitalRead(DI2);
   air1_status = Tony.digitalRead(DI3);
   air1_alarm = Tony.digitalRead(DI4);
   air1_CB_Trip = Tony.digitalRead(DI5);
   air1_Low_Pressure = Tony.digitalRead(DI6);
   air1_High_Pressure = Tony.digitalRead(DI7);

   /// Air 2
   air2_auto_mode = Tony.digitalRead(DI8);
   air2_manual_mode = Tony.digitalRead(DI9);
   air2_status = Tony.digitalRead(DI10);
   air2_alarm = Tony.digitalRead(DI11);
   air2_CB_Trip = Tony.digitalRead(DI12);
   air2_Low_Pressure = Tony.digitalRead(DI13);
   air2_High_Pressure = Tony.digitalRead(DI14);

   ///// Alarm
   fire_Alarm = Tony.digitalRead(DI15);

   //// Analog Read
   Room_Temp = Tony.analogRead(AI1);
   Room_Humidity = Tony.analogRead(AI2);

  if ( air1_auto_mode == HIGH && air2_auto_mode == HIGH)
  {
    Tony.digitalWrite(DO2,LOW);
    Tony.digitalWrite(DO1,HIGH);
  }
  if ( air1_auto_mode == HIGH && air2_auto_mode == LOW)
  {
    Tony.digitalWrite(DO1,HIGH);
  }
  if ( air1_auto_mode == LOW && air2_auto_mode == HIGH)
  {
    Tony.digitalWrite(DO2,HIGH);
  }
}
