#include <i2c_t3.h>
#include <VL53L0X.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;


VL53L0X sensor;
VL53L0X sensor2;


const int TrigPin = 23;                              //hc-sr04 pins
const int EchoPin = 22; 
float cm;
int  tonepin=21;
int i=0;

void setup()
{

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);                 //vl53l0x pins
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);

    pinMode(tonepin,OUTPUT);

  delay(500);
  drv.begin();
  
  Wire2.begin();                    //vl53l0x  i2c

  drv.selectLibrary(1);  
                                              // drv2605효과 설정
  drv.setMode(DRV2605_MODE_INTTRIG); 
  

  pinMode(TrigPin, OUTPUT); 
  pinMode(EchoPin, INPUT);               //초음파

  Serial.begin (9600);

  pinMode(9, INPUT);
  delay(150);
  Serial.println("00");
  sensor.init(true);
  sensor.setTimeout(500);

  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);            //vl53l0x   i2c주소 변경
  Serial.println("02");

  pinMode(10, INPUT);
    delay(150);
  sensor2.init(true);
  sensor2.setTimeout(500);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");
  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  sensor2.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);        
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  #if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
   sensor2.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY                   //  vl53l0x 모드 선택
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
  sensor2.setMeasurementTimingBudget(200000);
  #endif
  


}

void loop()
{
  Serial.print("-------상=");
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
   
   Serial.print("-------하=");
  Serial.print(sensor2.readRangeSingleMillimeters());
  if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  

  digitalWrite(TrigPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(TrigPin, HIGH); 
  delayMicroseconds(10);                          
  digitalWrite(TrigPin, LOW); 
                                                   //초음파
  cm = pulseIn(EchoPin, HIGH) / 58.0; //cm
  cm = (int(cm * 100.0)) / 100.0; //.00
  Serial.print("------초음파=");
  Serial.print(cm);
  Serial.println("");

 
  if(sensor2.readRangeSingleMillimeters()<1200&&cm>200){
         for(i=0;i<100;i++){      
          digitalWrite(tonepin,HIGH);
          delay(1);
          digitalWrite(tonepin,LOW);
          delay(1);
          }
         if(400<sensor2.readRangeSingleMillimeters()&&sensor2.readRangeSingleMillimeters()<800){
           drv.setWaveform(0,47);
           delay(700);
           drv.go(0);
           drv.go(1);
           }
       
          else if(sensor2.readRangeSingleMillimeters()<400){
            drv.setWaveform(0,47);
            delay(300);
            drv.go(0);
            drv.go(1);                                                 //하단에 막힌 경우
            }
      
       else{
        drv.setWaveform(0,47);
        delay(1000);
        drv.go(0);
        drv.go(1);
        }
        
  }
 else if(cm<120){
       if(sensor2.readRangeSingleMillimeters()<1200&& cm <180){   
              for(i=0;i<100;i++){      
                digitalWrite(tonepin,HIGH);
                delay(2);
                digitalWrite(tonepin,LOW);
                delay(2);
                }  
        
          if(700<sensor.readRangeSingleMillimeters()&&sensor.readRangeSingleMillimeters()<1200&&400<sensor2.readRangeSingleMillimeters()&&sensor2.readRangeSingleMillimeters()<800&& cm <180){
            drv.setWaveform(0,64);
            delay(700);
            drv.go(0);
            drv.go(1);
            }
      
          else if(sensor.readRangeSingleMillimeters()<700&&sensor2.readRangeSingleMillimeters()<400&& cm <180){
           
            drv.setWaveform(0,64);
            delay(200);                                        //모두가 막힌 경우
            drv.go(0);
            drv.go(1);                
            }
          else{
            drv.setWaveform(0,64);
            delay(1000);
            drv.go(0);
            drv.go(1);
         }
       }
       
      else {
         for(i=0;i<100;i++){      
          digitalWrite(tonepin,HIGH);
          delay(3);
          digitalWrite(tonepin,LOW);
          delay(3);
         }
        if(50<cm<80){
          drv.setWaveform(0,52);   
          drv.go(0);
          drv.go(1);
          delay(800);
          }
         else if(cm<30){
          drv.setWaveform(0,52);   
          drv.go(0);
          drv.go(1);
          delay(400);
         }
         else{
         drv.setWaveform(0,52);   
         drv.go(0);
         drv.go(1);
         delay(1300);
          }
      }
    
     
 } 
 else if(sensor.readRangeSingleMillimeters()<1200&&cm>200){
               for(i=0;i<100;i++){      
                digitalWrite(tonepin,HIGH);
                delay(4);
                digitalWrite(tonepin,LOW);
                delay(4);
                }
              if(400<sensor.readRangeSingleMillimeters()&&sensor.readRangeSingleMillimeters()<800){

                  drv.setWaveform(0,58);
                  delay(500);
                  drv.go(0);
                  drv.go(1);
                  }
            
               else if(sensor.readRangeSingleMillimeters()<400){
                 drv.setWaveform(0,58);   
                 drv.go(0);
                 drv.go(1);                                                   //위의 에  막힌 경우
                 delay(220);
                 }
           
               else{
                drv.setWaveform(0,58);
                delay(1000);
                drv.go(0);
                drv.go(1);
                }  
      } 
    
  
  delay(1);
  

}



