#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include "EasyNextionLibrary.h"
#include <ModbusMaster.h>

#define MAX485_DE      4
#define MAX485_RE_NEG  5

ModbusMaster node;

const int Air1_cb = 22, Air1_alarm = 23, Air1_low_pressure = 24, Air1_high_pressure = 25;
const int Air2_cb = 26, Air2_alarm = 27, Air2_low_pressure = 28, Air2_high_pressure = 29;
const int Fire1_alarm = 30 , Fire2_alarm = 31;
const int Ac_source_fail = 32 , Dc_source_fail = 33;
const int Air1_status = 34, Air2_status = 35;
const int Air1_auto_status = 36 , Air1_manual_status = 37;
const int Air2_auto_status = 38 , Air2_manual_status = 39;
const int Air1_onoff_control = 40, Air2_onoff_control = 41;
const int Dc_fan1_low = 42 ,Dc_fan1_medium = 43,Dc_fan1_high = 44;
const int Dc_fan2_low = 45 ,Dc_fan2_medium = 46,Dc_fan2_high = 47;
const int Alarm_output = 48;

float temp=0.0,humid=0.0,Dc_temp_on=0.0,Dc_step =0.0;
int temp_nextion = 0 , humid_nextion = 0;
uint8_t result; /// rs485
uint16_t data[2]; /// rs485
int t_on = 0 , t_off = 0, h_on = 0,h_off =0; //// receive from web

unsigned long previous_time = 0;
unsigned long air1_time_count = 100000,air2_time_count = 100000;
unsigned long air1_time_manual_count = 0,air2_time_manual_count = 0;

bool Air1_cb_on = false,Air1_alarm_on = false,Air1_low_on = false, Air1_high_on = false;
bool Air2_cb_on = false,Air2_alarm_on = false,Air2_low_on = false, Air2_high_on = false;

bool Air1_auto_status_on =false, Air1_manual_status_on= false,air1_auto = false;
bool Air2_auto_status_on =false, Air2_manual_status_on= false,air2_auto = false;
bool Air1_auto_status_work = true , Air2_auto_status_work = true;

bool alarm_for_air1 = false,alarm_for_air2 = false;

bool fire_alarm = false , Fire1_alarm_on = false , Fire2_alarm_on = false;
bool Air1_status_on = false, Air2_status_on = false;

bool Ac_source_fail_on = false , Dc_source_fail_on = false;

bool button_nextion_press = false, newPageLoaded = false;

EasyNex myNex(Serial3);

ros::NodeHandle nh;

std_msgs::Int8 RAir1_cb,RAir1_alarm,RAir1_low_pressure,RAir1_high_pressure;
std_msgs::Int8 RAir2_cb,RAir2_alarm,RAir2_low_pressure,RAir2_high_pressure;
std_msgs::Int8 RAir1_status,RAir2_status ,RAir1_auto,RAir2_auto,RAir1_manual,RAir2_manual;
std_msgs::Int8 Rair_duty_time,Rair_temp_on,Rair_temp_off,Rair_humid_on,Rair_humid_off,Rdc_fan_temp_on;
std_msgs::Float32 Rtemp,Rhumid,Rdc_fan_step_offset;

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
ros::Publisher humid_ros("humid_ros",&Rhumid);

ros::Publisher air1autostatus("Air1_auto_status",&RAir1_auto);
ros::Publisher air2autostatus("Air2_auto_status",&RAir2_auto);

ros::Publisher air1manualstatus("Air1_manual_status",&RAir1_manual);
ros::Publisher air2manualstatus("Air2_manual_status",&RAir2_manual);

ros::Publisher airdutytime("Air_duty_time",&Rair_duty_time);
ros::Publisher airtempon("Air_temp_on",&Rair_temp_on);
ros::Publisher airtempoff("Air_temp_off",&Rair_temp_off);
ros::Publisher airhumidon("Air_humid_on",&Rair_humid_on);
ros::Publisher airhumidoff("Air_humid_off",&Rair_humid_off);
ros::Publisher dcfantempon("Dc_fan_temp_on",&Rdc_fan_temp_on);
ros::Publisher dcfanstep("Dc_fan_step",&Rdc_fan_step_offset);
   
void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setting_time( const std_msgs::Int8 &time_air ){
    int h = time_air.data;
    air1_time_count = h*60*60;
    air2_time_count = h*60*60;
    myNex.writeNum("n0.val",h);
}
void setting_temp_on( const std_msgs::Int8& temp_on_air){
     t_on = temp_on_air.data;
    myNex.writeNum("n1.val",t_on);
}
void setting_temp_off( const std_msgs::Int8& temp_off_air){
     t_off = temp_off_air.data;
    myNex.writeNum("n2.val",t_off);
}
void setting_humid_on( const std_msgs::Int8& humid_on_air){
    int h_on = humid_on_air.data;

    myNex.writeNum("n3.val",h_on);
}
void setting_humid_off( const std_msgs::Int8& humid_off_air){
    int h_off = humid_off_air.data;
    myNex.writeNum("n4.val",h_off);
}
void setting_dcfan_temp_on( const std_msgs::Int8& dcfan_temp_on){
    Dc_temp_on = dcfan_temp_on.data;
    int dc_temp_on = dcfan_temp_on.data;
    myNex.writeNum("n5.val",dc_temp_on);
}
void setting_dcfan_step( const std_msgs::Float32& dcfan_step){
    Dc_step = dcfan_step.data;
    int dc_step = (dcfan_step.data)*10;
    myNex.writeNum("x2.val",dc_step);
}

ros::Subscriber<std_msgs::Int8> Rtimer_air("set_time", &setting_time);
ros::Subscriber<std_msgs::Int8> Rset_temp_on("set_temp_on", &setting_temp_on);
ros::Subscriber<std_msgs::Int8> Rset_temp_off("set_temp_off", &setting_temp_off);
ros::Subscriber<std_msgs::Int8> Rset_humid_on("set_humid_on", &setting_humid_on);
ros::Subscriber<std_msgs::Int8> Rset_humid_off("set_humid_off", &setting_humid_off);
ros::Subscriber<std_msgs::Int8> Rset_dcfan_temp_on("set_dcfan_temp_on", &setting_dcfan_temp_on);
ros::Subscriber<std_msgs::Float32> Rset_dcfan_step("set_dcfan_step", &setting_dcfan_step);


void setup() {
//  Serial.begin(9600);
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
   nh.advertise(air1status);
   nh.advertise(air2status);
   nh.advertise(air1autostatus);
   nh.advertise(air2autostatus);
   nh.advertise(air1manualstatus);
   nh.advertise(air2manualstatus);
   /////
   nh.subscribe(Rtimer_air);
   nh.subscribe(Rset_temp_on);
   nh.subscribe(Rset_temp_off);
   nh.subscribe(Rset_humid_on);
   nh.subscribe(Rset_humid_off);
   nh.subscribe(Rset_dcfan_temp_on);
   nh.subscribe(Rset_dcfan_step);
   /////
   nh.advertise(temp_ros);
   nh.advertise(humid_ros);
  ///// setting
   nh.advertise(airdutytime);
   nh.advertise(airtempon);
   nh.advertise(airtempoff);
   nh.advertise(airhumidon);
   nh.advertise(airhumidoff);
   nh.advertise(dcfantempon);
   nh.advertise(dcfanstep);

   ///// nextion
   myNex.begin(115200);
   delay(500);
  myNex.writeStr("page 0"); // For synchronizing Nextion page in case of reset to Arduino
  delay(50);
  myNex.lastCurrentPageId = 1;

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
  pinMode(Air1_auto_status,INPUT_PULLUP);
  pinMode(Air1_manual_status,INPUT_PULLUP);
  pinMode(Air2_auto_status,INPUT_PULLUP);
  pinMode(Air2_manual_status,INPUT_PULLUP);
  // Air On/Off Control
  pinMode(Air1_onoff_control,OUTPUT);
  pinMode(Air2_onoff_control,OUTPUT);
  digitalWrite(Air1_onoff_control,HIGH);
  digitalWrite(Air2_onoff_control,HIGH);

  //// Fire Alarm
  pinMode(Fire1_alarm,INPUT);
  pinMode(Fire2_alarm,INPUT);
  // DC fan 1
  digitalWrite(Dc_fan1_low,HIGH);
  digitalWrite(Dc_fan1_medium,HIGH);
  digitalWrite(Dc_fan1_high,HIGH);
  pinMode(Dc_fan1_low,OUTPUT);
  pinMode(Dc_fan1_medium,OUTPUT);
  pinMode(Dc_fan1_high,OUTPUT);
  // DC fan 2
  digitalWrite(Dc_fan2_low,HIGH);
  digitalWrite(Dc_fan2_medium,HIGH);
  digitalWrite(Dc_fan2_high,HIGH);
  pinMode(Dc_fan2_low,OUTPUT);
  pinMode(Dc_fan2_medium,OUTPUT);
  pinMode(Dc_fan2_high,OUTPUT);
  // Alarm output
  pinMode(Alarm_output,OUTPUT);

  ///// RS485
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  Serial1.begin(4800);
  node.begin(1, Serial1);  
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  
  ///// temp & humid RS485
  result = node.readHoldingRegisters(0, 2);
  if (result == node.ku8MBSuccess)
  {
    temp = node.getResponseBuffer(1)/10.0f;
    humid = node.getResponseBuffer(0)/10.0f;
  }
  
   temp_nextion = temp*10;
   humid_nextion = humid*10; 
   myNex.NextionListen();
   firstRefresh();
   String button_nextion_data = myNex.readStr("b15.txt");
   if(button_nextion_data == "Sended")
   {
    button_nextion_press = true;
   }
   if(button_nextion_press == true)
   {
     Rair_duty_time.data = myNex.readNumber("n0.val");
     Rair_temp_on.data = myNex.readNumber("n1.val");
     Rair_temp_off.data = myNex.readNumber("n2.val");
     Rair_humid_on.data = myNex.readNumber("n3.val");
     Rair_humid_off.data = myNex.readNumber("n4.val");
     Rdc_fan_temp_on.data = myNex.readNumber("n5.val");
     Rdc_fan_step_offset.data = myNex.readNumber("x2.val")/10.0f;

     Dc_temp_on = Rdc_fan_temp_on.data;
     Dc_step = Rdc_fan_step_offset.data;
    
     airdutytime.publish(&Rair_duty_time);
     airtempon.publish(&Rair_temp_on);
     airtempoff.publish(&Rair_temp_off);
     airhumidon.publish(&Rair_humid_on);
     airhumidoff.publish(&Rair_humid_off);
     dcfantempon.publish(&Rdc_fan_temp_on);
     dcfanstep.publish(&Rdc_fan_step_offset);
     button_nextion_press = false;
   }
   ///// timer
  if ( millis()-previous_time >= 1000)
  {
    air1_time_count = air1_time_count - 1 ;
    air2_time_count = air2_time_count - 1 ;
    myNex.writeNum("x0.val",temp_nextion);
    myNex.writeNum("x1.val",humid_nextion);
    Rtemp.data = temp;
    Rhumid.data = humid;
    temp_ros.publish(&Rtemp);
    humid_ros.publish(&Rhumid);
    previous_time = millis();
    
  }

  ///// Fire Alarm
//  if(digitalRead(Fire1_alarm) == HIGH && Fire1_alarm_on == true)
//  {
//    myNex.writeStr("t4.txt","Alarm");
//    myNex.writeNum("t4.pco",63488); // set color to red
//    Fire1_alarm_on = false;
//    fire_alarm = true;
//  }
//  if(digitalRead(Fire2_alarm) == HIGH && Fire2_alarm_on == true)
  //  { `
//    myNex.writeStr("t4.txt","Alarm");
//    myNex.writeNum("t4.pco",63488);
//    Fire2_alarm_on = false;
//    fire_alarm = true;
//  }
//  if(digitalRead(Fire1_alarm) == LOW && Fire1_alarm_on == false)
//  {
//    myNex.writeStr("t4.txt","Normal");
//    myNex.writeNum("t4.pco",65535); // set color to white
//    Fire1_alarm_on = true;
//    fire_alarm = false;
//  }
//  if(digitalRead(Fire2_alarm) == LOW && Fire2_alarm_on == false)
//  {
//    myNex.writeStr("t4.txt","Normal");
//    myNex.writeNum("t4.pco",65535);
//    Fire2_alarm_on = true;
//    fire_alarm = false;
//  }
  
    ///// Air1,Air2 Status (PULL_UP) change HIGH to LOW
  if(digitalRead(Air1_status) == LOW && Air1_status_on == true)
  {
     RAir1_status.data = digitalRead(Air1_status);
     air1status.publish(&RAir1_status);
     myNex.writeNum("p4.pic",1);
     Air1_status_on = false;
  }
  if(digitalRead(Air1_status) == HIGH && Air1_status_on == false)
  {
     RAir1_status.data = digitalRead(Air1_status);
     air1status.publish(&RAir1_status);
     myNex.writeNum("p4.pic",2);
     Air1_status_on = true;
  }
  if(digitalRead(Air2_status) == LOW && Air2_status_on == true)
  {
     RAir2_status.data = digitalRead(Air2_status);
     air2status.publish(&RAir2_status);
     myNex.writeNum("p5.pic",1);
     Air2_status_on = false;
  }
  if(digitalRead(Air2_status) == HIGH && Air2_status_on == false)
  {
     RAir2_status.data = digitalRead(Air2_status);
     air2status.publish(&RAir2_status);
     myNex.writeNum("p5.pic",2);
     Air2_status_on = true;
  }
  
    ///// Air1 cb alarm high low pressure on
    
  if(digitalRead(Air1_cb) == HIGH && Air1_cb_on == true)
    {
      RAir1_cb.data = digitalRead(Air1_cb);
      air1cb.publish(&RAir1_cb);
      myNex.writeNum("p8.pic",0);
      alarm_for_air1 = true;
      Air1_cb_on = false;
    }   
    if(digitalRead(Air1_alarm) == LOW && Air1_alarm_on == true)
    {
      RAir1_alarm.data = digitalRead(Air1_alarm);
      air1alarm.publish(&RAir1_alarm);
      myNex.writeNum("p6.pic",0);
      alarm_for_air1 = true;
      Air1_alarm_on = false;
    }
    
    if(digitalRead(Air1_low_pressure) == HIGH && Air1_low_on == true)
    {
      RAir1_low_pressure.data = digitalRead(Air1_low_pressure);
      air1low.publish(&RAir1_low_pressure);
      myNex.writeNum("p12.pic",0);
      alarm_for_air1 = true;
      Air1_low_on = false;
    }
    if(digitalRead(Air1_high_pressure) == HIGH && Air1_high_on == true)
    {
      RAir1_high_pressure.data = digitalRead(Air1_high_pressure);
      air1high.publish(&RAir1_high_pressure);
      myNex.writeNum("p10.pic",0);
      alarm_for_air1 = true;
      Air1_high_on = false;
    }
        
    ///// Air1 cb alarm low high pressure off
    
    if(digitalRead(Air1_cb) == LOW && Air1_cb_on == false)
   {
      RAir1_cb.data = digitalRead(Air1_cb);
      air1cb.publish(&RAir1_cb);
      myNex.writeNum("p8.pic",2);
      alarm_for_air1 = false;
      Air1_cb_on = true;
   }
    if(digitalRead(Air1_alarm) == HIGH && Air1_alarm_on == false)
    {
      RAir1_alarm.data = digitalRead(Air1_alarm);
      air1alarm.publish(&RAir1_alarm);
      myNex.writeNum("p6.pic",2);
      alarm_for_air1 = false;
      Air1_alarm_on = true;
    }
     if(digitalRead(Air1_low_pressure) == LOW && Air1_low_on == false)
    {
      RAir1_low_pressure.data = digitalRead(Air1_low_pressure);
      air1low.publish(&RAir1_low_pressure);
      myNex.writeNum("p12.pic",2);
      alarm_for_air1 = false;
      Air1_low_on = true;
    }
    if(digitalRead(Air1_high_pressure) == LOW && Air1_high_on == false)
    {
      RAir1_high_pressure.data = digitalRead(Air1_high_pressure);
      air1high.publish(&RAir1_high_pressure);
      myNex.writeNum("p10.pic",2);
      alarm_for_air1 = false;
      Air1_high_on = true;
    }
      
    //// Air2 cb alarm high low pressure on
    
    if(digitalRead(Air2_cb) == HIGH && Air2_cb_on == true)
    {
      RAir2_cb.data = digitalRead(Air2_cb);
      air2cb.publish(&RAir2_cb);
      myNex.writeNum("p9.pic",0);
      alarm_for_air2 = true;
      Air2_cb_on = false;
    }   
    if(digitalRead(Air2_alarm) == LOW && Air2_alarm_on == true)
    {
      RAir2_alarm.data = digitalRead(Air2_alarm);
      air2alarm.publish(&RAir2_alarm);
      myNex.writeNum("p7.pic",0);
      alarm_for_air2 = true;
      Air2_alarm_on = false;
    }
    
    if(digitalRead(Air2_low_pressure) == HIGH && Air2_low_on == true)
    {
      RAir2_low_pressure.data = digitalRead(Air2_low_pressure);
      air2low.publish(&RAir2_low_pressure);
      myNex.writeNum("p13.pic",0);
      alarm_for_air2 = true;
      Air2_low_on = false;
    }
    if(digitalRead(Air2_high_pressure) == HIGH && Air2_high_on == true)
    {
      RAir2_high_pressure.data = digitalRead(Air2_high_pressure);
      air2high.publish(&RAir2_high_pressure);
      myNex.writeNum("p11.pic",0);
      alarm_for_air2 = true;
      Air2_high_on = false;
    }
      
    ////// Air2 cb alarm high low pressure off
      
    if(digitalRead(Air2_cb) == LOW && Air2_cb_on == false)
   {
      RAir2_cb.data = digitalRead(Air2_cb);
      air2cb.publish(&RAir2_cb);
      myNex.writeNum("p9.pic",2);
      alarm_for_air2 = false;
      Air2_cb_on = true;
   }
    if(digitalRead(Air2_alarm) == HIGH && Air2_alarm_on == false)
    {
      RAir2_alarm.data = digitalRead(Air2_alarm);
      air2alarm.publish(&RAir2_alarm);
      myNex.writeNum("p7.pic",2);
      alarm_for_air2 = false;
      Air2_alarm_on = true;
    }
     if(digitalRead(Air2_low_pressure) == LOW && Air2_low_on == false)
    {
      RAir2_low_pressure.data = digitalRead(Air2_low_pressure);
      air2low.publish(&RAir2_low_pressure);
      myNex.writeNum("p13.pic",2);
      alarm_for_air2 = false;
      Air2_low_on = true;
    }
    if(digitalRead(Air2_high_pressure) == LOW && Air2_high_on == false)
    {
      RAir2_high_pressure.data = digitalRead(Air2_high_pressure);
      air2high.publish(&RAir2_high_pressure);
      myNex.writeNum("p11.pic",2);
      alarm_for_air2 = false;
      Air2_high_on = true;
    }

    ///// AC , DC Source fail
    if(digitalRead(Ac_source_fail) == HIGH && Ac_source_fail_on == true)
    {
      myNex.writeStr("t2.txt","Off");
      myNex.writeNum("t2.pco",63488); /// red color
      Ac_source_fail_on = false;
    }
    if(digitalRead(Dc_source_fail) == HIGH && Dc_source_fail_on == true)
    {
      myNex.writeStr("t3.txt","Off");
      myNex.writeNum("t3.pco",63488); /// red color
      Dc_source_fail_on = false;
    }
    if(digitalRead(Ac_source_fail) == LOW && Ac_source_fail_on == false)
    {
      myNex.writeStr("t2.txt","On");
      myNex.writeNum("t2.pco",2032); /// green color
      Ac_source_fail_on = true;
    }
    if(digitalRead(Dc_source_fail) == LOW && Dc_source_fail_on == false)
    {
      myNex.writeStr("t3.txt","On");
      myNex.writeNum("t3.pco",2032); /// green color
      Dc_source_fail_on = true;
    }      
      
      ////// Air1 Auto
  if( alarm_for_air1 == false && fire_alarm == false && temp >= t_on && humid >= h_on)
    {                  
      if(digitalRead(Air1_auto_status) == LOW && Air1_auto_status_work == true)
      {
         RAir1_auto.data = digitalRead(Air1_auto_status);
         air1autostatus.publish(&RAir1_auto);
         Air1_auto_status_on = true;
         Air1_auto_status_work = false;
         myNex.writeNum("p0.pic",1);
         air1_auto = true;
         air2_auto = false;
      }
      if(air1_time_count > 0 && Air1_auto_status_on == true && air1_auto == true) 
      {
        digitalWrite(Air1_onoff_control,LOW); // active relay cuase NC
        digitalWrite(Air2_onoff_control,HIGH);
      }
      if (air1_time_count == 0 && Air1_auto_status_on == true)
      {
        air1_time_count = 4*60*60;
        air1_auto = false;
        air2_auto = true;
        digitalWrite(Air1_onoff_control,HIGH);
      }
      if(digitalRead(Air1_auto_status) == HIGH && Air1_auto_status_work == false)
      {
         RAir1_auto.data = digitalRead(Air1_auto_status);
         air1autostatus.publish(&RAir1_auto);
         Air1_auto_status_work = true;
         myNex.writeNum("p0.pic",2);
      }
    }

  ///// Air2 Auto
  if ( alarm_for_air2 == false && fire_alarm == false && temp >= t_on && humid >= h_on)
  {
      if(digitalRead(Air2_auto_status) == LOW && Air2_auto_status_work == true)
      {
        RAir2_auto.data = digitalRead(Air2_auto_status);
        air2autostatus.publish(&RAir2_auto);
        
        Air2_auto_status_on = true;
        Air2_auto_status_work = false;
        myNex.writeNum("p1.pic",1);
      }
      if (air2_time_count > 0 && Air2_auto_status_on == true && air2_auto == true)
      {
        digitalWrite(Air2_onoff_control,LOW);
        digitalWrite(Air1_onoff_control,HIGH);
      }
      if (air2_time_count == 0 && Air2_auto_status_on == true)
      {
        air2_time_count = 4*60*60;
        air2_auto = false;
        air1_auto = true;
        digitalWrite(Air2_onoff_control,HIGH);
      }
      if(digitalRead(Air2_auto_status) == HIGH && Air2_auto_status_work == false)
      {
        RAir2_auto.data = digitalRead(Air2_auto_status);
        air2autostatus.publish(&RAir2_auto);
         Air2_auto_status_work = true;
         myNex.writeNum("p1.pic",2);
      }
  }
 
  ///// air1 manual
  if(digitalRead(Air1_manual_status) == LOW && Air1_manual_status_on == true)
  {
    RAir1_manual.data = digitalRead(Air1_manual_status);
    air1manualstatus.publish(&RAir1_manual);
    
    Air1_manual_status_on = false;
    myNex.writeNum("p2.pic",1);
  }
  if(digitalRead(Air1_manual_status) == HIGH && Air1_manual_status_on == false)
  {
    RAir1_manual.data = digitalRead(Air1_manual_status);
    air1manualstatus.publish(&RAir1_manual); 
       
    Air1_manual_status_on = false;
    myNex.writeNum("p2.pic",2);
  }
  //// air2 manual
  if(digitalRead(Air2_manual_status) == LOW && Air2_manual_status_on == true)
  {
    RAir2_manual.data = digitalRead(Air2_manual_status);
    air2manualstatus.publish(&RAir2_manual);
    
    Air2_manual_status_on = false;
    myNex.writeNum("p3.pic",1);
  }
  if(digitalRead(Air2_manual_status) == HIGH && Air2_manual_status_on == false)
  {
    RAir2_manual.data = digitalRead(Air2_manual_status);
    air2manualstatus.publish(&RAir2_manual);
        
    Air2_manual_status_on = true;
    myNex.writeNum("p3.pic",2);
  }

     ////// if alarm turn off one and turn on another && Alarm_output
   if (alarm_for_air1 == true)
   {
    digitalWrite(Air1_onoff_control,HIGH);
    digitalWrite(Air2_onoff_control,LOW); ///// LOW == active
    air2_auto = true;
    digitalWrite(Alarm_output,HIGH);
    
   }
   if (alarm_for_air2 == true)
   {
    digitalWrite(Air1_onoff_control,LOW);
    digitalWrite(Air2_onoff_control,HIGH);
    air1_auto = true;
    digitalWrite(Alarm_output,HIGH);
   }
   
  //// fire alram
  if(fire_alarm == true)
  {
    digitalWrite(Air1_onoff_control,HIGH); ///// PULL_UP HIGH == OFF
    digitalWrite(Air2_onoff_control,HIGH);
  }
 
 /////// turn on DC fan codition
  if (temp > Dc_temp_on)
  {
    digitalWrite(Dc_fan1_low,LOW);
    myNex.writeStr("t5.txt","Low");
    myNex.writeNum("t5.pco",34784); /// green color
  }
  if (temp > (Dc_temp_on +(Dc_step*1)))
  {
    digitalWrite(Dc_fan1_medium,LOW);
    myNex.writeStr("t5.txt","Medium");
    myNex.writeNum("t5.pco",65504); /// yellow color
  }
  if (temp > (Dc_temp_on +(Dc_step*2)))
  {
    digitalWrite(Dc_fan1_high,LOW);
    myNex.writeStr("t5.txt","High"); /// DC fan1
    myNex.writeNum("t5.pco",63488); /// red color
  }

  
  ////// turn off DC fan codition
  if(temp < (Dc_temp_on -(Dc_step*1)))
  {
    digitalWrite(Dc_fan1_low,HIGH);
    myNex.writeStr("t5.txt","Off"); // DC fan1
    myNex.writeNum("t5.pco",1055); /// blue color 
  }
  if(temp < Dc_temp_on)
  {
    digitalWrite(Dc_fan1_medium,HIGH);
  }
  if(temp < (Dc_temp_on +(Dc_step*1)))
  {
    digitalWrite(Dc_fan1_high,HIGH);
  }

 if (temp <= t_off || humid <= h_off)
 {
  digitalWrite(Air1_onoff_control,HIGH);
  digitalWrite(Air2_onoff_control,HIGH);
 }
  
    /////// Dc fan1
//  if( temp < 27)
//  {
//    digitalWrite(Dc_fan1_low,HIGH); // turn off 
//    myNex.writeStr("t5.txt","Off"); // DC fan1
//    myNex.writeNum("t5.pco",1055); /// blue color   
//  }
//  if( temp > 27 && temp <= 27.5)
//  {
//    digitalWrite(Dc_fan1_low,LOW); // turn on
//    digitalWrite(Dc_fan1_medium,HIGH);
//    digitalWrite(Dc_fan1_high,HIGH);
//    
//    myNex.writeStr("t5.txt","Low");
//    myNex.writeNum("t5.pco",34784); /// green color
//  }
//  if( temp > 27.5 && temp <= 28)
//  {
//    digitalWrite(Dc_fan1_low,HIGH);
//    digitalWrite(Dc_fan1_medium,LOW);
//    digitalWrite(Dc_fan1_high,HIGH);
//    
//    myNex.writeStr("t5.txt","Medium");
//    myNex.writeNum("t5.pco",65504); /// yellow color
//  }
//  if( temp > 28 && temp <= 28.5)
//  {
//    digitalWrite(Dc_fan1_low,HIGH);
//    digitalWrite(Dc_fan1_medium,HIGH);
//    digitalWrite(Dc_fan1_high,LOW);        
//    myNex.writeStr("t5.txt","High"); /// DC fan1
//    myNex.writeNum("t5.pco",63488); /// red color
//    digitalWrite(Dc_fan2_low,HIGH);
//    myNex.writeStr("t6.txt","Off"); // DC fan2
//    myNex.writeNum("t6.pco",1055); /// blue color
//  }
//  
//    ///// Dc fan2
//  if( temp > 28.5 && temp <= 29)
//  {
//    digitalWrite(Dc_fan2_low,LOW);
//    digitalWrite(Dc_fan2_medium,HIGH);
//    digitalWrite(Dc_fan2_high,HIGH);
//
//    myNex.writeStr("t6.txt","Low");
//    myNex.writeNum("t6.pco",34784); /// green color
//  }
//  if( temp > 29 && temp <= 29.5)
//  {
//    digitalWrite(Dc_fan2_low,HIGH);
//    digitalWrite(Dc_fan2_medium,LOW);
//    digitalWrite(Dc_fan2_high,HIGH);
//    myNex.writeStr("t5.txt","Medium");
//    myNex.writeNum("t5.pco",65504); /// yellow color
//  }
//  if( temp > 29.5)
//  {
//    digitalWrite(Dc_fan2_medium,HIGH);
//    digitalWrite(Dc_fan2_high,HIGH);
//    digitalWrite(Dc_fan2_high,LOW);
//    
//    myNex.writeStr("t5.txt","High");
//    myNex.writeNum("t5.pco",63488); /// red color    
//  }

  
  nh.spinOnce();

}

//void firstRefresh(){ // This function's purpose is to update the values of a new page when is first loaded,
//                     // and refreshing all the components with the current values as Nextion shows the Attribute val.
//
//  if(myNex.currentPageId != myNex.lastCurrentPageId) { // If the two variables are different, means a new page is loaded.
//    
//    newPageLoaded = true;    // A new page is loaded
//                             // This variable is used as an argument at the if() statement on the refreshPageXX() voids, 
//                             // in order when is true to update all the values on the page with their current values
//                             // with out run a comparison with the last value.
//
////    if(myNex.currentPageId == 0)
////    {
////      refreshPage0();
////    }
////    if(myNex.currentPageId == 1)
////    {
////      refreshPage1();
////    }
//    switch(myNex.currentPageId){
//      case 0:
//        refreshPage0();
//        break;
//      
//      case 1:
//        refreshPage1();
//        break;
//    }
//    
////    newPageLoaded = false;  // After we have updated the new page for the first time, we update the variable to false.
//                            // Now the values updated ONLY if the new value is different from the last Sent value.
//                            // See void refreshPage0()
//    
//    myNex.lastCurrentPageId = myNex.currentPageId; // Afer the refresh of the new page We make them equal,
//                                                   // in order to identify the next page change.
//  }
//}
//
//void refreshPage0(){
//  
//  if(newPageLoaded == true)
//  {
//    myNex.writeStr("t0.txt","test");
////    Serial.println("tested");
//    if(RAir1_status.data == 0)
//    {
//      myNex.writeNum("p4.pic",1);
//    }
//    if(RAir1_status.data == 1)
//    {
//      myNex.writeNum("p4.pic",2);
//    }
//    if(RAir2_status.data == 0)
//    {
//      myNex.writeNum("p5.pic",1);
//    }
//    if(RAir2_status.data == 1)
//    {
//      myNex.writeNum("p5.pic",2);
//    }
//    newPageLoaded = false;
//  }
//}
//
//void refreshPage1(){
//  // Use lastSentTemperature, in order to update the components ONLY when their value has changed 
//  // and avoid sending unnecessary data over Serial.
//  // Also with the newPageLoaded boolean variable, we bypass the if() argument of temperature != lastSentTemperature (last value comparison)
//  // so as to update all the values on Nextion when a new page is loaded, independant of if the values have changed
//  
//  if(newPageLoaded == true)
//  {
////    if(RAir1_status.data == 0)
////    {
////      myNex.writeNum("p4.pic",1);
////    }
//  }
//}
