#include<Wire.h>
#include<I2Cdev.h>
#include<MPU6050.h>
#include<PID_v1.h>

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
 
#define pin1 3 //for one pin of motor
#define pin2 5 // for another pin of motor

double sp,setpoint,input,output,out,val,in,sal;
PID mypid(&in,&out,&sp,2,15,18.5,DIRECT);
void setup() {
  
//delay(1000);
sp=0;
setpoint=analogRead(0);
Serial.begin(
  115200);
mypid.SetSampleTime(20);
mypid.SetMode(AUTOMATIC);
mypid.SetOutputLimits(-255,255);
 Serial.println("Initialize MPU");
 mpu.initialize();
 //Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
 pinMode(pin1,OUTPUT);
 pinMode(pin2,OUTPUT);
}

void loop() {
   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   double bx,setpoint,input,output;
   double ki,kp,kd;
   ax = map(ax, -17000, 17000, -255, 255 //ax is the tilt we are taking to change
   );// changing range
   Serial.print("ax = ");
   Serial.println(ax);
  setpoint=ax; 
  in=ax;
  Serial.print("in = ");
  Serial.println(in);
  val=setpoint-input;
  in=val; 
  mypid.Compute();
  Serial.print("val = ");
  Serial.println(val);
  if(abs(val)>0){
    if(val>0){
      output=abs(out);
    analogWrite(pin2,output);
    analogWrite(pin1,0);
    Serial.println("first");
    } 
    else{
      output=abs(out);
      analogWrite(pin1,output);
      analogWrite(pin2,0);
      Serial.println("second");
      }
    /*else {
      analogWrite(pin2,255);
      analogWrite(pin1,0);
      }*/
  }
  else{
    analogWrite(pin1,0);
    analogWrite(pin2,0);
    Serial.println("Third");
    }

  /*if(val<0){
    if(val>-255){
    analogWrite(pin1,-output);
    analogWrite(pin2,0);
    }
    else{
      analogWrite(pin1,255);
      analogWrite(pin2,0);
      }
  }*/
  delay(100);
 }
