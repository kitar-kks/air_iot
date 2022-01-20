#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include "EasyNextionLibrary.h"

//These values are in the datasheet
#define RT0 10000   // Ω
#define B 3977      // K
//--------------------------------------

#define VCC 5    //Supply voltage
#define R 10000  //R=10KΩ

//Variables
float RT, VR, ln, TX, T0, VRT;

const int Air1_cb = 31, Air1_alarm = 33, Air1_low_pressure = 30, Air1_high_pressure = 28;
const int Air2_cb = 23, Air2_alarm = 25, Air2_low_pressure = 27, Air2_high_pressure = 29;
const int Fire1_alarm = 32 , Fire2_alarm = 26;
const int Ac_source_fail = 43 , Dc_source_fail = 45;
const int Air1_status = 6, Air2_status = 7;
const int Air1_auto_status = 49 , Air1_manual_status = 24;
const int Air2_auto_status = 47 , Air2_manual_status = 22;
const int Air1_onoff_control = 34, Air2_onoff_control = 36;
const int Dc_fan1_low = 38 ,Dc_fan1_medium = 40,Dc_fan1_high = 42;
const int Dc_fan2_low = 44 ,Dc_fan2_medium = 46,Dc_fan2_high = 48;
const int Alarm_output = 2;

unsigned long previous_time = 0;
unsigned long air1_time_count = 0,air2_time_count = 0;
unsigned long air1_time_manual_count = 0,air2_time_manual_count = 0;

//unsigned long pt_air1_cb_on = 0, pt_air1_cb_off = 0 , pt_air1_alarm_on = 0, pt_air1_alarm_off = 0, pt_air1_low_on =0, pt_air1_low_off = 0, pt_air1_high_on = 0,pt_air1_high_off=0;
bool Air1_cb_on = false,Air1_alarm_on = false,Air1_low_on = false, Air1_high_on = false;
bool Air2_cb_on = false,Air2_alarm_on = false,Air2_low_on = false, Air2_high_on = false;

bool Air1_auto_status_on =false, Air1_manual_status_on= false,air1_auto = false;
bool Air2_auto_status_on =false, Air2_manual_status_on= false,air2_auto = false;

bool alarm_for_air1 = false,alarm_for_air2 = false;

bool fire_alarm = false , Fire1_alarm_on = false , Fire2_alarm_on = false;
bool Air1_status_on = false, Air2_status_on = false;


EasyNex myNex(Serial3);

ros::NodeHandle nh;

std_msgs::Int8 RAir1_cb,RAir1_alarm,RAir1_low_pressure,RAir1_high_pressure;
std_msgs::Int8 RAir2_cb,RAir2_alarm,RAir2_low_pressure,RAir2_high_pressure;
std_msgs::Int8 RAir1_status,RAir2_status;
std_msgs::Float32 Rtemp;

ros::Publisher air1cb("Air1_cb", &RAir1_cb);
ros::Publisher air1alarm("Air1_alarm",&RAir1_alarm);
ros::Publisher air1low("Air1_low_pressure",&RAir1_low_pressure);
ros::Publisher air1high("Air1_high_pressure",&RAir1_high_pressure);

ros::Publisher air2cb("Air2_cb", &RAir2_cb);
ros::Publisher air2alarm("Air2_alarm",&RAir2_alarm);
ros::Publisher air2low("Air2_low_pressure",&RAir2_low_pressure);
ros::Publisher air2high("Air2_high_pressure",&RAir2_high_pressure);

ros::Publisher air1status("Air1_status",&RAir1_status);
ros::Publisher air2status("Air2_status",&RAir2_status);

ros::Publisher temp_ros("temp_ros",&Rtemp);

void setting_time( const std_msgs::Int8& time_air){
    int h = time_air.data;
    air1_time_count = h*60*60;
    myNex.writeNum("h4.val",h);
    myNex.writeNum("n4.val",h);
//  nh.loginfo(time_air.data);
}
ros::Subscriber<std_msgs::Int8> timer_air("set_time", &setting_time);

void setup() {
//  Serial.begin(115200);
   nh.initNode();
   nh.advertise(air1cb);
   nh.advertise(air1alarm);
   nh.advertise(air1low);
   nh.advertise(air1high);
   /////
   nh.advertise(air2cb);
   nh.advertise(air2alarm);
   nh.advertise(air2low);
   nh.advertise(air2high);
   /////
   nh.advertise(temp_ros);
   nh.advertise(air1status);
   nh.advertise(air2status);
   /////
   nh.subscribe(timer_air);

   ///// nextion
   myNex.begin(115200);
   delay(500);  
   /////
   T0 = 25 + 273.15; //Temperature T0 from datasheet, conversion from Celsius to kelvin
  // Air1
  pinMode(Air1_cb,INPUT_PULLUP);
  pinMode(Air1_alarm,INPUT);
  pinMode(Air1_low_pressure,INPUT_PULLUP);
  pinMode(Air1_high_pressure,INPUT_PULLUP);
  // Air2
  pinMode(Air2_cb,INPUT_PULLUP);
  pinMode(Air2_alarm,INPUT);
  pinMode(Air2_low_pressure,INPUT_PULLUP);
  pinMode(Air2_high_pressure,INPUT_PULLUP);
  // source fail
  pinMode(Ac_source_fail,INPUT_PULLUP);
  pinMode(Dc_source_fail,INPUT_PULLUP);
  // Air Status
  pinMode(Air1_status,INPUT);
  pinMode(Air2_status,INPUT);
  // Air Auto/Manual Status
  pinMode(Air1_auto_status,INPUT);
  pinMode(Air1_manual_status,INPUT);
  pinMode(Air2_auto_status,INPUT);
  pinMode(Air2_manual_status,INPUT);
  // Air On/Off Control
  pinMode(Air1_onoff_control,OUTPUT);
  pinMode(Air2_onoff_control,OUTPUT);

  //// Fire Alarm
  pinMode(Fire1_alarm,INPUT);
  pinMode(Fire2_alarm,INPUT);
  // DC fan 1
  pinMode(Dc_fan1_low,OUTPUT);
  pinMode(Dc_fan1_medium,OUTPUT);
  pinMode(Dc_fan1_high,OUTPUT);
  // DC fan 2
  pinMode(Dc_fan2_low,OUTPUT);
  pinMode(Dc_fan2_medium,OUTPUT);
  pinMode(Dc_fan2_high,OUTPUT);
  // Alarm output
  pinMode(Alarm_output,OUTPUT);

}

void loop() {
  VRT = analogRead(A0);              //Acquisition analog value of VRT
  VRT = (5.00 / 1023.00) * VRT;      //Conversion to voltage
  VR = VCC - VRT;
  RT = VRT / (VR / R);               //Resistance of RT
  ln = log(RT / RT0);
  TX = (1 / ((ln / B) + (1 / T0))); //Temperature from thermistor
  float temp = TX - 273.15;                 //Conversion to Celsius
  int temp_nextion = temp*10;
  
  if ( millis()-previous_time >= 1000)
  {
    air1_time_count = air1_time_count - 1 ;
    air2_time_count = air2_time_count - 1 ;
    myNex.writeNum("x0.val",temp_nextion);
    Rtemp.data = temp;
    temp_ros.publish(&Rtemp);
    previous_time = millis();
    
  }

  ///// Fire Alarm
  if(digitalRead(Fire1_alarm) == HIGH && Fire1_alarm_on == true)
  {
    myNex.writeNum("p10.pic",1);
    myNex.writeStr("t4.txt","Alarm");
    myNex.writeNum("t4.pco",63488); // set color to red
    Fire1_alarm_on = false;
    fire_alarm = true;
  }
  if(digitalRead(Fire2_alarm) == HIGH && Fire2_alarm_on == true)
  {
    myNex.writeNum("p10.pic",1);
    myNex.writeStr("t4.txt","Alarm");
    myNex.writeNum("t4.pco",63488);
    Fire2_alarm_on = false;
    fire_alarm = true;
  }
  if(digitalRead(Fire1_alarm) == LOW && Fire1_alarm_on == false)
  {
    myNex.writeNum("p10.pic",3);
    myNex.writeStr("t4.txt","Normal");
    myNex.writeNum("t4.pco",65535); // set color to white
    Fire1_alarm_on = true;
    fire_alarm = false;
  }
  if(digitalRead(Fire2_alarm) == LOW && Fire2_alarm_on == false)
  {
    myNex.writeNum("p10.pic",3);
    myNex.writeStr("t4.txt","Normal");
    myNex.writeNum("t4.pco",65535);
    Fire2_alarm_on = true;
    fire_alarm = false;
  }
  
    ///// Air1,Air2 Status
  if(digitalRead(Air1_status) == HIGH && Air1_status_on == true)
  {
     RAir1_status.data = digitalRead(Air1_status);
     air1status.publish(&RAir1_status);
     myNex.writeNum("p0.pic",2);
     Air1_status_on = false;
  }
  if(digitalRead(Air1_status) == LOW && Air1_status_on == false)
  {
     RAir1_status.data = digitalRead(Air1_status);
     air1status.publish(&RAir1_status);
     myNex.writeNum("p0.pic",1);
     Air1_status_on = true;
  }
  if(digitalRead(Air2_status) == HIGH && Air2_status_on == true)
  {
     RAir2_status.data = digitalRead(Air2_status);
     air2status.publish(&RAir2_status);
     myNex.writeNum("p5.pic",2);
     Air2_status_on = false;
  }
  if(digitalRead(Air2_status) == LOW && Air2_status_on == false)
  {
     RAir2_status.data = digitalRead(Air2_status);
     air2status.publish(&RAir2_status);
     myNex.writeNum("p5.pic",1);
     Air2_status_on = true;
  }
  
    ///// Air1 cb alarm high low pressure on
    
  if(digitalRead(Air1_cb) == HIGH && Air1_cb_on == true)
    {
      RAir1_cb.data = digitalRead(Air1_cb);
      air1cb.publish(&RAir1_cb);
      myNex.writeNum("p2.pic",1);
      alarm_for_air1 = true;
      Air1_cb_on = false;
    }   
    if(digitalRead(Air1_alarm) == HIGH && Air1_alarm_on == true)
    {
      RAir1_alarm.data = digitalRead(Air1_alarm);
      air1alarm.publish(&RAir1_alarm);
      myNex.writeNum("p1.pic",1);
      alarm_for_air1 = true;
      Air1_alarm_on = false;
    }
    
    if(digitalRead(Air1_low_pressure) == HIGH && Air1_low_on == true)
    {
      RAir1_low_pressure.data = digitalRead(Air1_low_pressure);
      air1low.publish(&RAir1_low_pressure);
      myNex.writeNum("p4.pic",1);
      alarm_for_air1 = true;
      Air1_low_on = false;
    }
    if(digitalRead(Air1_high_pressure) == HIGH && Air1_high_on == true)
    {
      RAir1_high_pressure.data = digitalRead(Air1_high_pressure);
      air1high.publish(&RAir1_high_pressure);
      myNex.writeNum("p3.pic",1);
      alarm_for_air1 = true;
      Air1_high_on = false;
    }
        
    ///// Air1 cb alarm low high pressure off
    
    if(digitalRead(Air1_cb) == LOW && Air1_cb_on == false)
   {
      RAir1_cb.data = digitalRead(Air1_cb);
      air1cb.publish(&RAir1_cb);
      myNex.writeNum("p2.pic",3);
      alarm_for_air1 = false;
      Air1_cb_on = true;
   }
    if(digitalRead(Air1_alarm) == LOW && Air1_alarm_on == false)
    {
      RAir1_alarm.data = digitalRead(Air1_alarm);
      air1alarm.publish(&RAir1_alarm);
      myNex.writeNum("p1.pic",3);
      alarm_for_air1 = false;
      Air1_alarm_on = true;
    }
     if(digitalRead(Air1_low_pressure) == LOW && Air1_low_on == false)
    {
      RAir1_low_pressure.data = digitalRead(Air1_low_pressure);
      air1low.publish(&RAir1_low_pressure);
      myNex.writeNum("p4.pic",3);
      alarm_for_air1 = false;
      Air1_low_on = true;
    }
    if(digitalRead(Air1_high_pressure) == LOW && Air1_high_on == false)
    {
      RAir1_high_pressure.data = digitalRead(Air1_high_pressure);
      air1high.publish(&RAir1_high_pressure);
      myNex.writeNum("p3.pic",3);
      alarm_for_air1 = false;
      Air1_high_on = true;
    }
      
    //// Air2 cb alarm high low pressure on
    
    if(digitalRead(Air2_cb) == HIGH && Air2_cb_on == true)
    {
      RAir2_cb.data = digitalRead(Air2_cb);
      air2cb.publish(&RAir2_cb);
      myNex.writeNum("p7.pic",1);
      alarm_for_air2 = true;
      Air2_cb_on = false;
    }   
    if(digitalRead(Air2_alarm) == HIGH && Air2_alarm_on == true)
    {
      RAir2_alarm.data = digitalRead(Air2_alarm);
      air2alarm.publish(&RAir2_alarm);
      myNex.writeNum("p6.pic",1);
      alarm_for_air2 = true;
      Air2_alarm_on = false;
    }
    
    if(digitalRead(Air2_low_pressure) == HIGH && Air2_low_on == true)
    {
      RAir2_low_pressure.data = digitalRead(Air2_low_pressure);
      air2low.publish(&RAir2_low_pressure);
      myNex.writeNum("p9.pic",1);
      alarm_for_air2 = true;
      Air2_low_on = false;
    }
    if(digitalRead(Air2_high_pressure) == HIGH && Air2_high_on == true)
    {
      RAir2_high_pressure.data = digitalRead(Air2_high_pressure);
      air2high.publish(&RAir2_high_pressure);
      myNex.writeNum("p8.pic",1);
      alarm_for_air2 = true;
      Air2_high_on = false;
    }
      
    ////// Air2 cb alarm high low pressure off
      
    if(digitalRead(Air2_cb) == LOW && Air2_cb_on == false)
   {
      RAir2_cb.data = digitalRead(Air2_cb);
      air2cb.publish(&RAir2_cb);
      myNex.writeNum("p7.pic",3);
      alarm_for_air2 = false;
      Air2_cb_on = true;
   }
    if(digitalRead(Air2_alarm) == LOW && Air2_alarm_on == false)
    {
      RAir2_alarm.data = digitalRead(Air2_alarm);
      air2alarm.publish(&RAir2_alarm);
      myNex.writeNum("p6.pic",3);
      alarm_for_air2 = false;
      Air2_alarm_on = true;
    }
     if(digitalRead(Air2_low_pressure) == LOW && Air2_low_on == false)
    {
      RAir2_low_pressure.data = digitalRead(Air2_low_pressure);
      air2low.publish(&RAir2_low_pressure);
      myNex.writeNum("p9.pic",3);
      alarm_for_air2 = false;
      Air2_low_on = true;
    }
    if(digitalRead(Air2_high_pressure) == LOW && Air2_high_on == false)
    {
      RAir2_high_pressure.data = digitalRead(Air2_high_pressure);
      air2high.publish(&RAir2_high_pressure);
      myNex.writeNum("p8.pic",3);
      alarm_for_air2 = false;
      Air2_high_on = true;
    }
      
      ////// Air1 Auto
  if( alarm_for_air1 == false && fire_alarm == false)
    {                  
      if(digitalRead(Air1_auto_status) == HIGH)
      {
         Air1_auto_status_on = true;
         air1_auto = true;
         air2_auto = false;
      }
      if( air1_time_count > 0 && Air1_auto_status_on == true && air1_auto == true) 
      {
        digitalWrite(Air1_onoff_control,HIGH);
      }
      if (air1_time_count == 0 && Air1_auto_status_on == true)
      {
        air1_time_count = 6*60*60;
        air1_auto = false;
        air2_auto = true;
        digitalWrite(Air1_onoff_control,LOW);
      }
    }

  ///// Air2 Auto
  if ( alarm_for_air2 == false && fire_alarm == false)
  {
      if(digitalRead(Air2_auto_status) == HIGH)
      {
        bool Air2_auto_status_on = true;
      }
      if (air2_time_count > 0 && Air2_auto_status_on == true && air2_auto == true)
      {
        digitalWrite(Air2_onoff_control,HIGH);
      }
      if (air2_time_count == 0 && Air2_auto_status_on == true)
      {
        air2_time_count = 6*60*60;
        air2_auto = false;
        air1_auto = true;
        digitalWrite(Air2_onoff_control,LOW);
      }
  }

   ////// if alarm turn off one and turn on another && Alarm_output
   if (alarm_for_air1 == true)
   {
    digitalWrite(Air1_onoff_control,LOW);
    air2_auto = true;
    digitalWrite(Alarm_output,HIGH);
    
   }
   if (alarm_for_air2 == true)
   {
    digitalWrite(Air2_onoff_control,LOW);
    air1_auto = true;
    digitalWrite(Alarm_output,HIGH);
   }
   if( alarm_for_air1 == true && alarm_for_air2 == true)
   {
    digitalWrite(Air1_onoff_control,LOW);
    digitalWrite(Air2_onoff_control,LOW);
    digitalWrite(Alarm_output,HIGH);
   }
   
  ///// air1 manual
  if(digitalRead(Air1_manual_status) == HIGH)
  {
    Air1_auto_status_on = false;
  }
  //// air2 manual
  if(digitalRead(Air2_manual_status) == HIGH)
  {
    Air2_auto_status_on = false;
  }

  //// fire alram
  if(fire_alarm == true)
  {
    digitalWrite(Air1_onoff_control,LOW);
    digitalWrite(Air2_onoff_control,LOW);
  }
    /////// Dc fan1
  if( temp > 27 && temp <= 27.5)
  {
    digitalWrite(Dc_fan1_low,HIGH);
    digitalWrite(Dc_fan1_medium,LOW);
    digitalWrite(Dc_fan1_high,LOW);
  }
  if( temp > 27.5 && temp <= 28)
  {
    digitalWrite(Dc_fan1_low,LOW);
    digitalWrite(Dc_fan1_medium,HIGH);
    digitalWrite(Dc_fan1_high,LOW);
  }
  if( temp > 28 && temp <= 28.5)
  {
    digitalWrite(Dc_fan1_low,LOW);
    digitalWrite(Dc_fan1_medium,LOW);
    digitalWrite(Dc_fan1_high,HIGH);
  }
  
    ///// Dc fan2
  if( temp > 28.5 && temp <= 29)
  {
    digitalWrite(Dc_fan2_low,HIGH);
    digitalWrite(Dc_fan2_medium,LOW);
    digitalWrite(Dc_fan2_high,LOW);
  }
  if( temp > 29 && temp <= 29.5)
  {
    digitalWrite(Dc_fan2_low,LOW);
    digitalWrite(Dc_fan2_medium,HIGH);
    digitalWrite(Dc_fan2_high,LOW);
  }
  if( temp > 29.5)
  {
    digitalWrite(Dc_fan2_medium,LOW);
    digitalWrite(Dc_fan2_high,LOW);
    digitalWrite(Dc_fan2_high,HIGH);
  }

  
  nh.spinOnce();

}
