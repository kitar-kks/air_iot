#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>


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
bool Air1_cb_on = true,Air1_alarm_on = true,Air1_low_on = true, Air1_high_on = true;
bool Air2_cb_on = true,Air2_alarm_on = true,Air2_low_on = true, Air2_high_on = true;

bool Air1_auto_status_on =false, Air1_manual_status_on= false,air1_auto = false;
bool Air2_auto_status_on =false, Air2_manual_status_on= false,air2_auto = false;

ros::NodeHandle nh;

std_msgs::Int8 RAir1_cb,RAir1_alarm,RAir1_low_pressure,RAir1_high_pressure;
std_msgs::Int8 RAir2_cb,RAir2_alarm,RAir2_low_pressure,RAir2_high_pressure;

ros::Publisher air1cb("Air1_cb", &RAir1_cb);
ros::Publisher air1alarm("Air1_alarm",&RAir1_alarm);
ros::Publisher air1low("Air1_low_pressure",&RAir1_low_pressure);
ros::Publisher air1high("Air1_high_pressure",&RAir1_high_pressure);

ros::Publisher air2cb("Air2_cb", &RAir2_cb);
ros::Publisher air2alarm("Air2_alarm",&RAir2_alarm);
ros::Publisher air2low("Air2_low_pressure",&RAir2_low_pressure);
ros::Publisher air2high("Air2_high_pressure",&RAir2_high_pressure);

void setting_time( const std_msgs::Int8& time_air){
    int h = time_air.data;
    air1_time_count = h*60*60;
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
   nh.subscribe(timer_air);
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
//  Serial.println("---------------------");
//  Serial.print("Air2_cb = ");
//  Serial.println(digitalRead(Air2_cb));
//  Serial.print("Air2_alarm = ");
//  Serial.println(digitalRead(Air2_alarm));
//  Serial.print("Air2_low = ");
//  Serial.println(digitalRead(Air2_low_pressure));
//  Serial.print("Air2_high = ");
//  Serial.println(digitalRead(Air2_high_pressure));
  
  if ( millis()-previous_time >= 1000)
  {
    air1_time_count = air1_time_count - 1 ;
    air2_time_count = air2_time_count - 1 ;
    previous_time = millis();
    
  }
    ///// Air1 cb alarm high low pressure on
    
  if(digitalRead(Air1_cb) == HIGH && Air1_cb_on == true)
    {
      RAir1_cb.data = digitalRead(Air1_cb);
      air1cb.publish(&RAir1_cb);
      Air1_cb_on = false;
    }   
    if(digitalRead(Air1_alarm) == HIGH && Air1_alarm_on == true)
    {
      RAir1_alarm.data = digitalRead(Air1_alarm);
      air1alarm.publish(&RAir1_alarm);
      Air1_alarm_on = false;
    }
    
    if(digitalRead(Air1_low_pressure) == HIGH && Air1_low_on == true)
    {
      RAir1_low_pressure.data = digitalRead(Air1_low_pressure);
      air1low.publish(&RAir1_low_pressure);
      Air1_low_on = false;
    }
    if(digitalRead(Air1_high_pressure) == HIGH && Air1_high_on == true)
    {
      RAir1_high_pressure.data = digitalRead(Air1_high_pressure);
      air1high.publish(&RAir1_high_pressure);
      Air1_high_on = false;
    }
        
    ///// Air1 cb alarm low high pressure off
    
    if(digitalRead(Air1_cb) == LOW && Air1_cb_on == false)
   {
      RAir1_cb.data = digitalRead(Air1_cb);
      air1cb.publish(&RAir1_cb);
      Air1_cb_on = true;
   }
    if(digitalRead(Air1_alarm) == LOW && Air1_alarm_on == false)
    {
      RAir1_alarm.data = digitalRead(Air1_alarm);
      air1alarm.publish(&RAir1_alarm);
      Air1_alarm_on = true;
    }
     if(digitalRead(Air1_low_pressure) == LOW && Air1_low_on == false)
    {
      RAir1_low_pressure.data = digitalRead(Air1_low_pressure);
      air1low.publish(&RAir1_low_pressure);
      Air1_low_on = true;
    }
    if(digitalRead(Air1_high_pressure) == LOW && Air1_high_on == false)
    {
      RAir1_high_pressure.data = digitalRead(Air1_high_pressure);
      air1high.publish(&RAir1_high_pressure);
      Air1_high_on = true;
    }
      
    //// Air2 cb alarm high low pressure on
    
    if(digitalRead(Air2_cb) == HIGH && Air2_cb_on == true)
    {
      RAir2_cb.data = digitalRead(Air2_cb);
      air2cb.publish(&RAir2_cb);
      Air2_cb_on = false;
    }   
    if(digitalRead(Air2_alarm) == HIGH && Air2_alarm_on == true)
    {
      RAir2_alarm.data = digitalRead(Air2_alarm);
      air2alarm.publish(&RAir2_alarm);
      Air2_alarm_on = false;
    }
    
    if(digitalRead(Air2_low_pressure) == HIGH && Air2_low_on == true)
    {
      RAir2_low_pressure.data = digitalRead(Air2_low_pressure);
      air2low.publish(&RAir2_low_pressure);
      Air2_low_on = false;
    }
    if(digitalRead(Air2_high_pressure) == HIGH && Air2_high_on == true)
    {
      RAir2_high_pressure.data = digitalRead(Air2_high_pressure);
      air2high.publish(&RAir2_high_pressure);
      Air2_high_on = false;
    }
      
    ////// Air2 cb alarm high low pressure off
      
    if(digitalRead(Air2_cb) == LOW && Air2_cb_on == false)
   {
      RAir2_cb.data = digitalRead(Air2_cb);
      air2cb.publish(&RAir2_cb);
      Air2_cb_on = true;
   }
    if(digitalRead(Air2_alarm) == LOW && Air2_alarm_on == false)
    {
      RAir2_alarm.data = digitalRead(Air2_alarm);
      air2alarm.publish(&RAir2_alarm);
      Air2_alarm_on = true;
    }
     if(digitalRead(Air2_low_pressure) == LOW && Air2_low_on == false)
    {
      RAir2_low_pressure.data = digitalRead(Air2_low_pressure);
      air2low.publish(&RAir2_low_pressure);
      Air2_low_on = true;
    }
    if(digitalRead(Air2_high_pressure) == LOW && Air2_high_on == false)
    {
      RAir2_high_pressure.data = digitalRead(Air2_high_pressure);
      air2high.publish(&RAir2_high_pressure);
      Air2_high_on = true;
    }
      
      ////// Air1 Auto
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
  
  ///// Air2 Auto
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


  
  nh.spinOnce();

}
