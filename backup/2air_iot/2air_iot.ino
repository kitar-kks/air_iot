#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16MultiArray.h>
#include "EasyNextionLibrary.h"
#include <ModbusMaster.h>
#include <avr/wdt.h>

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
const int fan_cpu = 10;

float temp = 0.0, humid = 0.0, Dc_temp_on = 30.0, Dc_step = 2.0,pi_temp=60.0;
int temp_nextion = 0 , humid_nextion = 0;
uint8_t result; /// rs485
uint16_t data[2]; /// rs485
int t_on = 30 , t_off = 24, h_on = 75,h_off =70; //// receive from web
int h = 4, interval = 1000; 

float dc_med=0.0 ,dc_high=0.0 ,dc_low_off=0.0 ,dc_high_off=0.0;

unsigned long previous_time = 0,pre_time_air1=0,pre_time_air2=0;
unsigned long air1_time_count = 60,air2_time_count = 60; ///// timer air
unsigned long air1_time_manual_count = 0,air2_time_manual_count = 0;

unsigned long time_Air1_status_on = 0, time_Air1_status_off = 0, time_Air2_status_on = 0, time_Air2_status_off = 0;

unsigned long time_Air1_cb_on = 0,time_Air1_alarm_on = 0,time_Air1_low_on = 0,time_Air1_high_on = 0;
unsigned long time_Air1_cb_off = 0,time_Air1_alarm_off = 0,time_Air1_low_off = 0,time_Air1_high_off = 0;

unsigned long time_Air2_cb_on = 0,time_Air2_alarm_on = 0,time_Air2_low_on = 0,time_Air2_high_on = 0;
unsigned long time_Air2_cb_off = 0,time_Air2_alarm_off = 0,time_Air2_low_off = 0,time_Air2_high_off = 0;

unsigned long time_Air1_auto_status_on = 0,time_Air1_auto_status_off = 0;
unsigned long time_Air2_auto_status_on = 0,time_Air2_auto_status_off = 0;

unsigned long time_Air1_manual_status_on = 0,time_Air1_manual_status_off = 0;
unsigned long time_Air2_manual_status_on = 0,time_Air2_manual_status_off = 0;

unsigned long time_dc_fan_low_on = 0, time_dc_fan_medium_on = 0,time_dc_fan_high_on = 0;
unsigned long time_dc_fan_low_off = 0, time_dc_fan_medium_off = 0,time_dc_fan_high_off = 0;

unsigned long time_Fire1_alarm_on = 0,time_Fire1_alarm_off = 0,time_Fire2_alarm_on = 0,time_Fire2_alarm_off = 0;
unsigned long time_ac_fail_on = 0,time_dc_fail_on = 0,time_ac_fail_off = 0,time_dc_fail_off = 0;
bool Air1_status_on = false, Air2_status_on = false;
bool Air1_cb_on = false,Air1_alarm_on = false,Air1_low_on = false, Air1_high_on = false;
bool Air2_cb_on = false,Air2_alarm_on = false,Air2_low_on = false, Air2_high_on = false;

bool Air1_auto_status_on =false, Air1_manual_status_on= false,air1_auto = false;
bool Air2_auto_status_on =false, Air2_manual_status_on= false,air2_auto = false;
bool Air1_auto_status_work = true , Air2_auto_status_work = true;

bool alarm_for_air1 = false,alarm_for_air1_cb = false,alarm_for_air1_alarm = false,alarm_for_air1_low = false,alarm_for_air1_high = false;
bool alarm_for_air2 = false,alarm_for_air2_cb = false,alarm_for_air2_alarm = false,alarm_for_air2_low = false,alarm_for_air2_high = false;

bool fire_alarm = false , Fire1_alarm_on = true , Fire2_alarm_on = true;

bool Ac_source_fail_on = false , Dc_source_fail_on = false;

bool button_nextion_press = false, newPageLoaded = false;
bool dc_fan_low_working = false,dc_fan_med_working = false, dc_fan_high_working = false;

EasyNex myNex(Serial3);

ros::NodeHandle nh;

std_msgs::Int8 Rdc_fan_temp_on;

std_msgs::Float32 Rtemp,Rhumid,Rdc_fan_step_offset;

std_msgs::Int16MultiArray Rsource_fail ,Rdc_fan ,RAir1_alarm,RAir2_alarm ,RAir1_status,RAir2_status ,RAir_setting;


ros::Publisher air1alarm("Air1_alarm",&RAir1_alarm);
ros::Publisher air2alarm("Air2_alarm",&RAir2_alarm);

ros::Publisher air1status("Air1_status",&RAir1_status);
ros::Publisher air2status("Air2_status",&RAir2_status);

ros::Publisher temp_ros("temp_ros",&Rtemp);
ros::Publisher humid_ros("humid_ros",&Rhumid);

ros::Publisher airsetting("Air_setting",&RAir_setting);

ros::Publisher dcfantempon("Dc_fan_temp_on",&Rdc_fan_temp_on);
ros::Publisher dcfanstep("Dc_fan_step",&Rdc_fan_step_offset);

ros::Publisher ac_dc_fail("source_fail",&Rsource_fail);
ros::Publisher dc_fan("dc_fan_status",&Rdc_fan);

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
     h = time_air.data;
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
void setting_temp_cpu_pi( const std_msgs::Float32& temp_pi){
    pi_temp = temp_pi.data;
}

ros::Subscriber<std_msgs::Int8> Rtimer_air("set_time", &setting_time);
ros::Subscriber<std_msgs::Int8> Rset_temp_on("set_temp_on", &setting_temp_on);
ros::Subscriber<std_msgs::Int8> Rset_temp_off("set_temp_off", &setting_temp_off);
ros::Subscriber<std_msgs::Int8> Rset_humid_on("set_humid_on", &setting_humid_on);
ros::Subscriber<std_msgs::Int8> Rset_humid_off("set_humid_off", &setting_humid_off);
ros::Subscriber<std_msgs::Int8> Rset_dcfan_temp_on("set_dcfan_temp_on", &setting_dcfan_temp_on);
ros::Subscriber<std_msgs::Float32> Rset_dcfan_step("set_dcfan_step", &setting_dcfan_step);

ros::Subscriber<std_msgs::Float32> temp_cpu_pi("set_temp_cpu_pi", &setting_temp_cpu_pi);

void setup() {
   nh.initNode();   
   /////
   nh.advertise(air1alarm);
   nh.advertise(air2alarm);
   /////
   RAir1_alarm.data_length = 5;
   RAir2_alarm.data_length = 5;
   
   RAir1_status.data_length = 4;
   RAir2_status.data_length = 4;
   
   RAir_setting.data_length = 6;
   
   Rsource_fail.data_length = 3;
   Rdc_fan.data_length = 3;
   
   ///// air status
   nh.advertise(air1status);
   nh.advertise(air2status);
   /////
   nh.advertise(temp_ros);
   nh.advertise(humid_ros);
    ///// setting
   nh.advertise(airsetting);
   nh.advertise(dcfantempon);
   nh.advertise(dcfanstep);
    ////// ac dc fail
   nh.advertise(ac_dc_fail);
   nh.advertise(dc_fan);
   
   /////
   nh.subscribe(Rtimer_air);
   nh.subscribe(Rset_temp_on);
   nh.subscribe(Rset_temp_off);
   nh.subscribe(Rset_humid_on);
   nh.subscribe(Rset_humid_off);
   nh.subscribe(Rset_dcfan_temp_on);
   nh.subscribe(Rset_dcfan_step);
   nh.subscribe(temp_cpu_pi);

   ///// nextion
   myNex.begin(115200);
   delay(500);
   myNex.writeStr("page 0"); // For synchronizing Nextion page in case of reset to Arduino
   delay(50);
   myNex.lastCurrentPageId = 1;
   ////// watch dog
   wdt_enable(WDTO_4S);
   
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

  pinMode(fan_cpu,OUTPUT);

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
   String button_nextion_data = myNex.readStr("b15.txt");
   if(button_nextion_data == "Sended")
   {
    button_nextion_press = true;
   }
   if(button_nextion_press == true)
   {
    ///// setting
     RAir_setting.data[1] = myNex.readNumber("n0.val");   // duty time
     RAir_setting.data[2] = myNex.readNumber("n1.val");    // temp on
     RAir_setting.data[3] = myNex.readNumber("n2.val");  // temp off
     RAir_setting.data[4] = myNex.readNumber("n3.val"); // humid on
     RAir_setting.data[5] = myNex.readNumber("n4.val"); // humid off
     Rdc_fan_temp_on.data = myNex.readNumber("n5.val"); // dc fan temp on
     Rdc_fan_step_offset.data = myNex.readNumber("x2.val")/10.0f; // dc fan step

     Dc_temp_on = Rdc_fan_temp_on.data;
     Dc_step = Rdc_fan_step_offset.data;

     h =  RAir_setting.data[1];
     air1_time_count = h*60*60;
     air2_time_count = h*60*60;
     t_on = RAir_setting.data[2];
     t_off = RAir_setting.data[3];
     h_on = RAir_setting.data[4];
     h_off = RAir_setting.data[5];
     
     airsetting.publish(&RAir_setting);
     
     dcfantempon.publish(&Rdc_fan_temp_on);
     dcfanstep.publish(&Rdc_fan_step_offset);
     button_nextion_press = false;
   }
   ///// timer
  if (millis()-previous_time >= 1000)
  {
    myNex.writeNum("x0.val",temp_nextion);
    myNex.writeNum("x1.val",humid_nextion);
    Rtemp.data = temp;
    Rhumid.data = humid;
    temp_ros.publish(&Rtemp);
    humid_ros.publish(&Rhumid);
    previous_time = millis();
    
  }
  if (millis() - pre_time_air1 >= 1000 && air1_auto == true)
  {
    air1_time_count = air1_time_count - 1 ;
    pre_time_air1 = millis();
  }
  if (millis() - pre_time_air2 >= 1000 && air2_auto == true)
  {
    air2_time_count = air2_time_count - 1 ;
    pre_time_air2 = millis();
  }

    ///// Fire Alarm 
  if(digitalRead(Fire1_alarm) == HIGH && Fire1_alarm_on == true) //// Normal (logic High)
  {
    myNex.writeStr("t4.txt","Normal");
    myNex.writeNum("t4.pco",65535); // set color to white
    Fire1_alarm_on = false;
  }
  if(digitalRead(Fire2_alarm) == HIGH && Fire2_alarm_on == true) //// Normal (logic High)
  { 
    myNex.writeStr("t4.txt","Normal");
    myNex.writeNum("t4.pco",65535);
    Fire2_alarm_on = false;
  }
  if(digitalRead(Fire1_alarm) == LOW && Fire1_alarm_on == false) //// Fire Alarm (logic Low)
  {
    myNex.writeStr("t4.txt","Alarm");
    myNex.writeNum("t4.pco",63488); // set color to red
    Fire1_alarm_on = true;
  }
  if(digitalRead(Fire2_alarm) == LOW && Fire2_alarm_on == false) //// Fire Alarm (logic Low)
  {
    myNex.writeStr("t4.txt","Alarm");
    myNex.writeNum("t4.pco",63488);
    Fire2_alarm_on = true;
  }
  if(Fire1_alarm_on == false && Fire2_alarm_on == false)
  {
    fire_alarm = false;
  }
  if(Fire1_alarm_on == true || Fire2_alarm_on == true)
  {
    fire_alarm = true;
  }

  ///// Fire_alarm = normal
       ///// Air1,Air2 Status (PULL_UP) change HIGH to LOW
    if(digitalRead(Air1_status) == LOW && millis()- time_Air1_status_on >= interval)
    {   
       RAir1_status.data[1] = digitalRead(Air1_status);
       air1status.publish(&RAir1_status);
       myNex.writeNum("p4.pic",1);
       Air1_status_on = false;
       time_Air1_status_on = millis();
           
    }
    if(digitalRead(Air1_status) == HIGH && millis()- time_Air1_status_off >= interval )
    {
       RAir1_status.data[1] = digitalRead(Air1_status);
       air1status.publish(&RAir1_status);
       myNex.writeNum("p4.pic",2);
       air1_auto = false;
       Air1_status_on = true;
       time_Air1_status_off = millis();
    }
    if(digitalRead(Air2_status) == LOW && millis()- time_Air2_status_on >= interval)
    {
       RAir2_status.data[1] = digitalRead(Air2_status);
       air2status.publish(&RAir2_status);
       myNex.writeNum("p5.pic",1);
       Air2_status_on = false;
       time_Air2_status_on = millis();
    }
    if(digitalRead(Air2_status) == HIGH && millis()- time_Air2_status_off >= interval)
    {
       RAir2_status.data[1] = digitalRead(Air2_status);
       air2status.publish(&RAir2_status);
       myNex.writeNum("p5.pic",2);
       air2_auto = false;
       Air2_status_on = true;
       time_Air2_status_off = millis();
    }
    
      ///// Air1 cb alarm high low pressure on
      
    if(digitalRead(Air1_cb) == HIGH && millis()- time_Air1_cb_on >= interval)
      {
        RAir1_alarm.data[1] = digitalRead(Air1_cb);
        air1alarm.publish(&RAir1_alarm);
        myNex.writeNum("p8.pic",0);
        //alarm_for_air1 = true;
        alarm_for_air1_cb = true;
        Air1_cb_on = false;
        time_Air1_cb_on = millis();
      }   
      if(digitalRead(Air1_alarm) == LOW && millis()- time_Air1_alarm_on >= interval)
      {
        RAir1_alarm.data[2] = digitalRead(Air1_alarm);
        air1alarm.publish(&RAir1_alarm);
        myNex.writeNum("p6.pic",0);
        alarm_for_air1_alarm = true;
        Air1_alarm_on = false;
        time_Air1_alarm_on = millis();
      }    
      if(digitalRead(Air1_low_pressure) == HIGH && millis()- time_Air1_low_on >= interval)
      {
        RAir1_alarm.data[3] = digitalRead(Air1_low_pressure);
        air1alarm.publish(&RAir1_alarm);
        myNex.writeNum("p12.pic",0);
        alarm_for_air1_low = true;
        Air1_low_on = false;
        time_Air1_low_on = millis();
      }
      if(digitalRead(Air1_high_pressure) == HIGH && millis()- time_Air1_high_on >= interval)
      {
        RAir1_alarm.data[4] = digitalRead(Air1_high_pressure);
        air1alarm.publish(&RAir1_alarm);
        myNex.writeNum("p10.pic",0);
        alarm_for_air1_high = true;
        Air1_high_on = false;
        time_Air1_high_on = millis();
      }
          
      ///// Air1 cb alarm low high pressure off
      
      if(digitalRead(Air1_cb) == LOW && millis()- time_Air1_cb_off >= interval)
     {
        RAir1_alarm.data[1] = digitalRead(Air1_cb);
        air1alarm.publish(&RAir1_alarm);
        myNex.writeNum("p8.pic",2);
        alarm_for_air1_cb = false;
        Air1_cb_on = true;
        time_Air1_cb_off = millis();
     }
      if(digitalRead(Air1_alarm) == HIGH && millis()- time_Air1_alarm_off >= interval)
      {
        RAir1_alarm.data[2] = digitalRead(Air1_alarm);
        air1alarm.publish(&RAir1_alarm);
        myNex.writeNum("p6.pic",2);
        alarm_for_air1_alarm = false;
        Air1_alarm_on = true;
        time_Air1_alarm_off = millis();
      }
       if(digitalRead(Air1_low_pressure) == LOW && millis()- time_Air1_low_off >= interval )
      {
        RAir1_alarm.data[3] = digitalRead(Air1_low_pressure);
        air1alarm.publish(&RAir1_alarm);
        myNex.writeNum("p12.pic",2);
        alarm_for_air1_low = false;
        Air1_low_on = true;
        time_Air1_low_off = millis();
      }
      if(digitalRead(Air1_high_pressure) == LOW && millis()- time_Air1_high_off >= interval )
      {
        RAir1_alarm.data[4] = digitalRead(Air1_high_pressure);
        air1alarm.publish(&RAir1_alarm);
        myNex.writeNum("p10.pic",2);
        alarm_for_air1_high = false;
        Air1_high_on = true;
        time_Air1_high_off = millis();
      }
        
      //// Air2 cb alarm high low pressure on
      
      if(digitalRead(Air2_cb) == HIGH && millis()- time_Air2_cb_on >= interval)
      {
        RAir2_alarm.data[1] = digitalRead(Air2_cb);
        air2alarm.publish(&RAir2_alarm);
        myNex.writeNum("p9.pic",0);
        alarm_for_air2_cb = true;
        Air2_cb_on = false;
        time_Air2_cb_on = millis();
      }   
      if(digitalRead(Air2_alarm) == LOW && millis()- time_Air2_alarm_on >= interval)
      {
        RAir2_alarm.data[2] = digitalRead(Air2_alarm);
        air2alarm.publish(&RAir2_alarm);
        myNex.writeNum("p7.pic",0);
        alarm_for_air2_alarm = true;
        Air2_alarm_on = false;
        time_Air2_alarm_on = millis();
      }
      
      if(digitalRead(Air2_low_pressure) == HIGH && millis()- time_Air2_low_on >= interval)
      {
        RAir2_alarm.data[3] = digitalRead(Air2_low_pressure);
        air2alarm.publish(&RAir2_alarm);
        myNex.writeNum("p13.pic",0);
        alarm_for_air2_low = true;
        Air2_low_on = false;
        time_Air2_low_on = millis();
      }
      if(digitalRead(Air2_high_pressure) == HIGH && millis()- time_Air2_high_on >= interval)
      {
        RAir2_alarm.data[4] = digitalRead(Air2_high_pressure);
        air2alarm.publish(&RAir2_alarm);
        myNex.writeNum("p11.pic",0);
        alarm_for_air2_high = true;
        Air2_high_on = false;
        time_Air2_high_on = millis();
      }
        
      ////// Air2 cb alarm high low pressure off
        
      if(digitalRead(Air2_cb) == LOW && millis()- time_Air2_cb_off >= interval)
     {
        RAir2_alarm.data[1] = digitalRead(Air2_cb);
        air2alarm.publish(&RAir2_alarm);
        myNex.writeNum("p9.pic",2);
        alarm_for_air2_cb = false;
        Air2_cb_on = true;
        time_Air2_cb_off = millis();
     }
      if(digitalRead(Air2_alarm) == HIGH && millis()- time_Air2_alarm_off >= interval )
      {
        RAir2_alarm.data[2] = digitalRead(Air2_alarm);
        air2alarm.publish(&RAir2_alarm);
        myNex.writeNum("p7.pic",2);
        alarm_for_air2_alarm = false;
        Air2_alarm_on = true;
        time_Air2_alarm_off = millis();
      }
       if(digitalRead(Air2_low_pressure) == LOW && millis()- time_Air2_low_off >= interval )
      {
        RAir2_alarm.data[3] = digitalRead(Air2_low_pressure);
        air2alarm.publish(&RAir2_alarm);
        myNex.writeNum("p13.pic",2);
        alarm_for_air2_low = false;
        Air2_low_on = true;
        time_Air2_low_off = millis();
      }
      if(digitalRead(Air2_high_pressure) == LOW && millis()- time_Air2_high_off >= interval )
      {
        RAir2_alarm.data[4] = digitalRead(Air2_high_pressure);
        air2alarm.publish(&RAir2_alarm);
        myNex.writeNum("p11.pic",2);
        alarm_for_air2_high = false;
        Air2_high_on = true;
        time_Air2_high_off = millis();
  
      }
  
      ///// AC , DC Source fail
      if(digitalRead(Ac_source_fail) == HIGH && millis() - time_ac_fail_off >= interval)
      {
        myNex.writeStr("t2.txt","Off");
        myNex.writeNum("t2.pco",63488); /// red color
        Rsource_fail.data[1] = digitalRead(Ac_source_fail);
        ac_dc_fail.publish(&Rsource_fail);
        Ac_source_fail_on = false;
        time_ac_fail_off = millis();
      }
      if(digitalRead(Dc_source_fail) == HIGH && millis() - time_dc_fail_off >= interval )
      {
        myNex.writeStr("t3.txt","Off");
        myNex.writeNum("t3.pco",63488); /// red color
        Rsource_fail.data[2] = digitalRead(Dc_source_fail);
        ac_dc_fail.publish(&Rsource_fail);
        Dc_source_fail_on = false;
        time_dc_fail_off = millis();
      }
      if(digitalRead(Ac_source_fail) == LOW && millis() - time_ac_fail_on >= interval )
      {
        myNex.writeStr("t2.txt","On");
        myNex.writeNum("t2.pco",2032); /// green color
        Rsource_fail.data[1] = digitalRead(Ac_source_fail);
        ac_dc_fail.publish(&Rsource_fail);
        Ac_source_fail_on = true;
        time_ac_fail_on = millis();
      }
      if(digitalRead(Dc_source_fail) == LOW && millis() - time_dc_fail_on >= interval)
      {
        myNex.writeStr("t3.txt","On");
        myNex.writeNum("t3.pco",2032); /// green color
        Rsource_fail.data[2] = digitalRead(Dc_source_fail);
        ac_dc_fail.publish(&Rsource_fail);
        Dc_source_fail_on = true;
        time_dc_fail_on = millis();
      }      

        ////// Air1 Auto
    if( alarm_for_air1 == false && fire_alarm == false)
      {                  
        if(digitalRead(Air1_auto_status) == LOW && millis() - time_Air1_auto_status_on >= interval)
        {
           RAir1_status.data[2] = digitalRead(Air1_auto_status);
           air1status.publish(&RAir1_status);
           Air1_auto_status_on = true;
           myNex.writeNum("p0.pic",1);
           if(Air1_auto_status_work == true)
           {
            air1_auto = true;
            air2_auto = false;
            Air1_auto_status_work = false;
           }
           time_Air1_auto_status_on = millis();
        }
        if(air1_time_count > 0 && Air1_auto_status_on == true && air1_auto == true) 
        {
          digitalWrite(Air1_onoff_control,LOW); // active relay cuase NC
  //        digitalWrite(Air2_onoff_control,HIGH);
        }
        if (air1_time_count == 0 && Air1_auto_status_on == true)
        {
          air1_time_count = 60;
//          myNex.writeStr("t6.txt","re t1");
          air1_auto = false;
          air2_auto = true;
          digitalWrite(Air1_onoff_control,HIGH);
        }
        if((temp >= t_on || humid >= h_on) && air1_auto == true)
        {
          digitalWrite(Air2_onoff_control,LOW);
          air2_auto = true;
        }
        if((temp <= t_off && humid <= h_off) && air1_auto == true)
        {
          digitalWrite(Air2_onoff_control,HIGH);
        }
        if(digitalRead(Air1_auto_status) == HIGH && millis() - time_Air1_auto_status_off >= interval) /// check air1 auto status off
        {
           RAir1_status.data[2] = digitalRead(Air1_auto_status);
           air1status.publish(&RAir1_status);
           myNex.writeNum("p0.pic",2);
           Air1_auto_status_on = false;
           Air1_auto_status_work = true;
           time_Air1_auto_status_off = millis();
        }
      }
  
    ///// Air2 Auto
    if ( alarm_for_air2 == false && fire_alarm == false )
    {
        if(digitalRead(Air2_auto_status) == LOW && millis() - time_Air2_auto_status_on >= interval)
        {
           RAir2_status.data[2] = digitalRead(Air2_auto_status);
           air2status.publish(&RAir2_status);
          myNex.writeNum("p1.pic",1);
          Air2_auto_status_on = true;
          time_Air2_auto_status_on = millis();
        }
        if (air2_time_count > 0 && Air2_auto_status_on == true && air2_auto == true)
        {
  //        digitalWrite(Air1_onoff_control,HIGH);
          digitalWrite(Air2_onoff_control,LOW);
        }
        if (air2_time_count == 0 && Air2_auto_status_on == true)
        {
          air2_time_count = 60;
//          myNex.writeStr("t6.txt","re t2");
          air1_auto = true;
          air2_auto = false; 
          Air1_auto_status_work = true;
          digitalWrite(Air2_onoff_control,HIGH);
        }
        if((temp >= t_on || humid >= h_on) && air2_auto == true)
        {
          digitalWrite(Air1_onoff_control,LOW);
          air1_auto = true;
        }
        if((temp <= t_off && humid <= h_off) && air2_auto == true)
        {
          digitalWrite(Air1_onoff_control,HIGH);
        }
        if(digitalRead(Air2_auto_status) == HIGH && millis() - time_Air2_auto_status_off >= interval)
        {
          RAir2_status.data[2] = digitalRead(Air2_auto_status);
          air2status.publish(&RAir2_status);
          myNex.writeNum("p1.pic",2);
          Air2_auto_status_on = false;
          time_Air2_auto_status_off = millis();
        }
    }
   
    ///// air1 manual
    if(digitalRead(Air1_manual_status) == LOW && millis() - time_Air1_manual_status_on >= interval)
    {
      RAir1_status.data[3] = digitalRead(Air1_manual_status);
      air1status.publish(&RAir1_status);
      myNex.writeNum("p2.pic",1);
      time_Air1_manual_status_on = millis();
    }
    if(digitalRead(Air1_manual_status) == HIGH && millis() - time_Air1_manual_status_off >= interval)
    {
      RAir1_status.data[3] = digitalRead(Air1_manual_status);
      air1status.publish(&RAir1_status);
      myNex.writeNum("p2.pic",2);
      time_Air1_manual_status_off = millis();
    }
    //// air2 manual
    if(digitalRead(Air2_manual_status) == LOW && millis() - time_Air2_manual_status_on >= interval)
    {
      RAir2_status.data[3] = digitalRead(Air2_manual_status);
      air2status.publish(&RAir2_status);
      myNex.writeNum("p3.pic",1);
      time_Air2_manual_status_on = millis();
    }
    if(digitalRead(Air2_manual_status) == HIGH && millis() - time_Air2_manual_status_off >= interval)
    {
      RAir2_status.data[3] = digitalRead(Air2_manual_status);
      air2status.publish(&RAir2_status);
      myNex.writeNum("p3.pic",2);
      time_Air2_manual_status_off = millis();
    }
  
       ////// if alarm turn off one and turn on another && Alarm_output
     if (alarm_for_air1_cb == true || alarm_for_air1_alarm == true || alarm_for_air1_low == true || alarm_for_air1_high == true)
     {
      digitalWrite(Air1_onoff_control,HIGH);
      digitalWrite(Air2_onoff_control,LOW); ///// LOW == active
      air2_auto = true;
      alarm_for_air1 = true;
      digitalWrite(Alarm_output,HIGH);   
     }
     if (alarm_for_air2_cb == true || alarm_for_air2_alarm == true || alarm_for_air2_low == true || alarm_for_air2_high == true)
     {
      digitalWrite(Air1_onoff_control,LOW);
      digitalWrite(Air2_onoff_control,HIGH);
      air1_auto = true;
      alarm_for_air2 = true;
      digitalWrite(Alarm_output,HIGH);
     }
  
     if (alarm_for_air1_cb == false && alarm_for_air1_alarm == false && alarm_for_air1_low == false && alarm_for_air1_high == false)
     {
      alarm_for_air1 = false;
     }
     if (alarm_for_air2_cb == false && alarm_for_air2_alarm == false && alarm_for_air2_low == false && alarm_for_air2_high == false)
     {
      alarm_for_air2 = false;
     }
  
     if(alarm_for_air1 == true && alarm_for_air2 == true)
     {
      digitalWrite(Air1_onoff_control,HIGH);
      digitalWrite(Air2_onoff_control,HIGH);
     }
    
   /////// turn on DC fan codition
   dc_med = (Dc_temp_on +(Dc_step*1));   // ex.temp_on = 30,step = 1 : 31
   dc_high = (Dc_temp_on +(Dc_step*2));  // 32
   
   dc_low_off = (Dc_temp_on -(Dc_step*1));  // 29
   dc_high_off = (Dc_temp_on +(Dc_step*1)); // 31
   
    if ((temp > Dc_temp_on && temp <= dc_med) && millis() - time_dc_fan_low_on >= interval)
    { 
      digitalWrite(Dc_fan1_low,LOW);
      digitalWrite(Dc_fan2_low,LOW);
      Rdc_fan.data[1] = 1;
      Rdc_fan.data[2] = 1;
      dc_fan.publish(&Rdc_fan);
      myNex.writeStr("t5.txt","Low");
      myNex.writeNum("t5.pco",34784); /// green color
      myNex.writeStr("t6.txt","Low");
      myNex.writeNum("t6.pco",34784); /// green color
            
      dc_fan_low_working = true;          
      time_dc_fan_low_on = millis();
    }
    if ((temp > dc_med && temp <= dc_high) && millis() - time_dc_fan_medium_on >= interval)
    {
      digitalWrite(Dc_fan1_medium,LOW);
      digitalWrite(Dc_fan2_medium,LOW);
      Rdc_fan.data[1] = 2;
      Rdc_fan.data[2] = 2;
//      Rdc_fan.data_length = 3; //// data[0] bug
      dc_fan.publish(&Rdc_fan);
      myNex.writeStr("t5.txt","Medium");
      myNex.writeNum("t5.pco",65504); /// yellow color
      myNex.writeStr("t6.txt","Medium");
      myNex.writeNum("t6.pco",65504); /// yellow color
      dc_fan_med_working == true;
      time_dc_fan_medium_on = millis();
    }    
    if (temp > dc_high && millis() - time_dc_fan_high_on >= interval)
    {
      digitalWrite(Dc_fan1_high,LOW);
      digitalWrite(Dc_fan2_high,LOW);
      Rdc_fan.data[1] = 3;
      Rdc_fan.data[2] = 3;
      dc_fan.publish(&Rdc_fan);
      myNex.writeStr("t5.txt","High"); /// DC fan1
      myNex.writeNum("t5.pco",63488); /// red color
      myNex.writeStr("t6.txt","High"); /// DC fan1
      myNex.writeNum("t6.pco",63488); /// red color
      dc_fan_high_working = true;
      time_dc_fan_high_on = millis();
    }
    
    ////// turn off DC fan codition
    if(temp < dc_low_off && millis() - time_dc_fan_low_off >= interval && dc_fan_low_working == true)
    {
      digitalWrite(Dc_fan1_low,HIGH);
      digitalWrite(Dc_fan2_low,HIGH);
      Rdc_fan.data[1] = 0;
      Rdc_fan.data[2] = 0;
      dc_fan.publish(&Rdc_fan);
      myNex.writeStr("t5.txt","Off"); // DC fan1
      myNex.writeNum("t5.pco",1055); /// blue color 
      myNex.writeStr("t6.txt","Off"); // DC fan1
      myNex.writeNum("t6.pco",1055); /// blue color
      time_dc_fan_low_off = millis(); 
      dc_fan_low_working = false;
    }
    if(temp < Dc_temp_on && millis() - time_dc_fan_medium_off >= interval && dc_fan_med_working == true)
    {
      digitalWrite(Dc_fan1_medium,HIGH);
      digitalWrite(Dc_fan2_medium,HIGH);
      Rdc_fan.data[1] = 0;
      Rdc_fan.data[2] = 0;
      dc_fan.publish(&Rdc_fan);
      myNex.writeStr("t5.txt","Off"); // DC fan1
      myNex.writeNum("t5.pco",1055); /// blue color 
      myNex.writeStr("t6.txt","Off"); // DC fan1
      myNex.writeNum("t6.pco",1055); /// blue color
      dc_fan_med_working = false;
      time_dc_fan_medium_off = millis ();
    }
    if(temp < dc_high_off && millis() - time_dc_fan_high_off >= interval && dc_fan_high_working == true)
    {
      digitalWrite(Dc_fan1_high,HIGH);
      digitalWrite(Dc_fan2_high,HIGH);
      Rdc_fan.data[1] = 0;
      Rdc_fan.data[2] = 0;
      dc_fan.publish(&Rdc_fan);
      myNex.writeStr("t5.txt","Off"); // DC fan1
      myNex.writeNum("t5.pco",1055); /// blue color 
      myNex.writeStr("t6.txt","Off"); // DC fan1
      myNex.writeNum("t6.pco",1055); /// blue color
      time_dc_fan_high_off = millis();
      dc_fan_high_working = false;
    }

    if (pi_temp >= 50.00) //// Pi
    {
//      digitalWrite(fan_cpu,LOW);
      analogWrite(fan_cpu,255);
    }

    //// fire alram
  if(fire_alarm == true)
  {
    digitalWrite(Air1_onoff_control,HIGH); ///// PULL_UP HIGH == OFF
    digitalWrite(Air2_onoff_control,HIGH);
    
    if(millis() - time_Fire1_alarm_on >= 1000)
    {
      myNex.writeStr("t4.txt","Alarm");
      myNex.writeNum("t4.pco",63488);
      time_Fire1_alarm_on = millis();    
    }
  }
  
  wdt_reset(); //// if clock delay more than 4s reset
  nh.spinOnce();

}
