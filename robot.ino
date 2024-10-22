#include<Stepper.h>
#include<Servo.h>
Servo m3;
Servo m2;

int count=0;
int pinm2=11;
int pinm3=12;

char gen;
int m3pos=0;
int m3max=180;
int m3min=0;
int incm3=20;
int m2pos=0;
int m2max=60;
int m2min=0;
int incm2=15;
int m1pos=1019;
int m1max=2038;
int m1min=0;
int incm1=250;
// Defines the number of steps per rotation
const int stepsPerRevolution = 2038;

// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence

Stepper m1 = Stepper(stepsPerRevolution, 2,4,3,5);
void setup() {
  m3.write(m3pos);
  m2.write(m2pos);
  Serial.begin(9600);
  m3.attach(pinm3);
  m2.attach(pinm2);
  m1.setSpeed(10);
	m1.step(m1pos);
  Serial.println("Input Mode No.:");
  while(1){ 
  if(Serial.available()>0){
  gen=Serial.read();
  Serial.println(gen);
  break;
  }
  }
  //Serial.println("hdrfufu,g");
while(1){//main while   
  switch (gen)//mode switch
  {
    case '1' : 
      Serial.println("Mode 1");  
      while(1){ 
      if(gen!='a'&&gen!='w'&&gen!='s'&&gen!='d'&&gen!='i'&&gen!='k'&&gen!='1'){
        break;
      }
      
      while(1){ 
        if(Serial.available()>0){
          gen=Serial.read();
          Serial.println(gen);
          break;
      }
      }

      switch (gen) 
      {
        case 'a':
          fm1(1);
          break;
        case 'd':
          fm1(0);
          break;
        case 's':
          fm2(1);
          break;
        case 'w':
          fm2(0);
          break;
        case 'i':
          fm3(1);
          break;
        case 'k':
          fm3(0);
          break;
        case '2':
          break;
      break;
      }
      }
    break;
    case '2' :   
    Serial.println("Mode 2");
    while(1){ 
        if(gen!='a'&&gen!='w'&&gen!='s'&&gen!='d'&&gen!='2'){
        break;
      }
      while(1){ 
        if(Serial.available()>0){
        gen=Serial.read();
        Serial.println(gen);
        break;
        }
      }
      switch (gen) //dir switch
      {
        case 'a':
          fm1(1);
          break;
        case 'd':
          fm1(0);
          break;
        case 's':
          fm2(1);
          switch (m2pos){
            case 0:
              m3.write(18);
              break;
            case 15:
              m3.write(33);
              break;
            case 30:
              m3.write(48);
              break;
            case 45:
              m3.write(63);
              break;
            case 60:
              m3.write(78);
              break;
                         
          }
          break;
        case 'w':
          fm2(0);
          switch (m2pos){
            case 0:
              m3.write(18);
              break;
            case 15:
              m3.write(33);
              break;
            case 30:
              m3.write(48);
              break;
            case 45:
              m3.write(63);
              break;
            case 60:
              m3.write(78);
              break;
                         
          }
          break;
      
      }
      
    }
      break;
    case '5' : 
        Serial.println("Tracking mode");  
        incm1 = 50;
        while(1){ 
          if(gen!='a'&&gen!='w'&&gen!='s'&&gen!='d'&&gen!='5'){
          incm1 = 250;
          break;
        }
        while(1){ 
          if(Serial.available()>0){
          gen=Serial.read();
          Serial.println(gen);
          break;
          } 
        }
        switch (gen) //dir switch
        {
          case 'a':
            fm1(1);
            break;
          case 'd':
            fm1(0);
            break;
          case 's':
            fm2(1);
            switch (m2pos){
              case 0:
                m3.write(18);
                break;
              case 15:
                m3.write(33);
                break;
              case 30:
                m3.write(48);
                break;
              case 45:
                m3.write(63);
                break;
              case 60:
                m3.write(78);
                break;
                         
            }
          break;
        case 'w':
          fm2(0);
          switch (m2pos){
            case 0:
              m3.write(18);
              break;
            case 15:
              m3.write(33);
              break;
            case 30:
              m3.write(48);
              break;
            case 45:
              m3.write(63);
              break;
            case 60:
              m3.write(78);
              break;
                         
          }
          break;
      
      }
      
    }
    break;
      case '3' :                                                                /////////////////////////////////////////////////////////////////////////
      Serial.println("Automatic mode");

      m3pos = 18;
      m2pos = 0;
      while(1){
        if(Serial.available()>0){
        gen=Serial.read();
        Serial.println(gen);
        if(gen!='3')
          break;
        } 
        fm1(1);
        if(m1pos==2019){
          while(1){
            fm1(0);
            if(m1pos==19)
              break;
          }
        }
        }
        break;
  }

      if(gen==4){
        break;
      }
}
    m3.write(0);
    
  Serial.println("Baklava");
  while(m1pos!=19){
    m1.setSpeed(10);
    m1pos=m1pos-incm1;
	  m1.step(-incm1);  
  }
    m1.setSpeed(10);
	  m1.step(-19); 

}

void loop() {

}
void fm3(int c){
  switch (c)
  {
    case 1:
      if(m3pos+incm3<=m3max){
        m3pos=m3pos+incm3;
        m3.write(m3pos);
      }
    break;
    case 0:
      if(m3pos-incm3>=m3min){
        m3pos=m3pos-incm3;
        m3.write(m3pos);
      }
    break;
  
  }
  m2.write(m2pos);
  Serial.println(m3pos);
}
void fm2(int c){
  switch (c)
  {
    case 1:
      if(m2pos+incm2<=m2max){
        m2pos=m2pos+incm2;
        m2.write(m2pos);
      }
      break;
    case 0:
      if(m2pos-incm2>=m2min){
        m2pos=m2pos-incm2;
        m2.write(m2pos);
      }
      break;
  
  }
  m3.write(m3pos);
  Serial.println(m2pos);  
}
void fm1(int c){
  switch (c)
  {
    case 1:
      if(m1pos+incm1<=2038){
          m1pos=m1pos+incm1;
          m1.setSpeed(10);
	        m1.step(incm1);
      }
      break;
    case 0:
      if(m1pos-incm1>=0){
          m1pos=m1pos-incm1;
          m1.setSpeed(10);
	        m1.step(-incm1);
      }
      break;
  }
}
