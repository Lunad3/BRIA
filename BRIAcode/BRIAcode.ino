
// SERVO
#include <Servo.h> 
Servo myservo1;  // create servo object to control a servo 
Servo myservo2; 
Servo myservo3; 
Servo myservo4; 
Servo myservo5; 
Servo myservo6;  
Servo myservo7; 
Servo myservo8;  

  
// MPU  FOR FUTURE TESTS (ADAPT THE KINEMATIC TO THE PELVIS MVT°
#include<Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float z_acc;
float x_acc;
float y_acc;
float y_gyro;
float x_gyro;
float z_gyro;
float angle_ap=0;
float angle_ml=0; 
float err=0;


int T=14;             // TIME BETWEEN EACH LOOP
int KSW=30;          //30  % SWING AMPLITUDE OF THE KNEE JOINT

int KML=20;          //20  // MEDIO LATERAL AMPLITUDE OF THE ANKLE
int KST=10;          //  10   // HIP FLEXION AMPLITUDE
int KSF=5;           // FOR FUTURE STANCE PHASE KNEE FLEXION
int t=0;             // % OF GAIT CYCLE FOR RIGHT LIMB
int ts=0;            // % OF GAIT CYCLE FOR LEFT LIMB


// SERIAL
String inputString = "";         // a string to hold incoming data
String value = "S";              // VALUE IS USED FOR MODE SELECTION S=STANDING, B= WALKING
String param ="";                // TO MODIFY A PARAMETER VIA SERIAL KST, T, KML, KSW, KSF
boolean stringComplete = false; 


// CALIB    MEANS THE POSTIONS OF THE SERVO DURING STANDING
int HG=100; //100        //DEFAULT POSITION OF THE LEFT HIP FLEXION (HG=HANCHE GAUCHE IN FRENCH°
int GG=20;              //  LEFT KNEE
int CG=85;   //85       LEFT ANGKE FLEXION
int HD=95;  //95        RIGHT HIP FLEXION
int GD=170 ;            // RIGHT KNEE
int CD=100;  //100      //  RIGHT ANKLE FLEXION
int FG=102;  //102      // LEFT ANKLE MEDIAL LATERAL
int FD=85;   // 85 EXTERNE   // RIGHT ANKLE MEDIAL LATERAL
int PD=50;                // FOR FUTURE HIP ROTATION OR HIP MEDIO LATERAL
int PG=50;


// MVT ARRAYS
int FHD[101];  //          HIP FLEXION (RIGHT) 
int FKD[101];  //          KNEE FLEXION (RIGHT) 
int FAD[101];  //          ANKLE FLEXION (RIGHT) 
int MAD[101];  //          ANKLE ML (RIGHT) 
  
  
void setup() 
{ 
pinMode(13,OUTPUT);
// SERVO
  myservo1.attach(2);  // HIP   FLEXION LEFT
  myservo2.attach(3);  // KNEE  FL EXION LEFT
  myservo3.attach(4);  // ANKLE FLEXION LEFT
  myservo4.attach(5);  // HIP   FLEXION RIGHT
  myservo5.attach(6);  // KNEE  FLEXION RIGHT
  myservo6.attach(7);  // ANKLE FLEXION RIGHT
  myservo7.attach(8);  // ANKLE ML      LEFT
  myservo8.attach(9);  // ANKLE ML      RIGHT
 
  
  
// MPU
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);  // PWR_MGMT_1 register
Wire.write(0);     // set to zero (wakes up the MPU-6050)
Wire.endTransmission(true);


//SERIAL
Serial.begin(9600);
inputString.reserve(200);
setkin(FHD,FKD,FAD,MAD,KST,KSW,KML);   // CALL THE ARRAYS SETTING WITH DEFAULT VALUES

} 
 
 
void loop() 
{ 
  
//PARAM MODIFICATION VIA SERIAL
   Serial.println(value);
   if (inputString.substring(0,1) == "T") {   //MODIFICATION OF TIME T
    Serial.println(inputString);      
    param=inputString.substring(1);
    Serial.println(param);
    T=param.toInt();
     stringComplete = false;
       inputString = "";
    } // fin de D 

   if (inputString.substring(0,1) == "M") {     // MODIFICATION OF KML
    Serial.println(inputString);      
    param=inputString.substring(1);
    Serial.println(param);
    KML=param.toInt();  
   setkin(FHD,FKD,FAD,MAD,KST,KSW,KML);
  stringComplete = false;
    inputString = "";
    } // fin de D 


   if (inputString.substring(0,1) == "P") {      // MODIF OF KST
    Serial.println(inputString);      
    param=inputString.substring(1);
    Serial.println(param);
    KST=param.toInt();
setkin(FHD,FKD,FAD,MAD,KST,KSW,KML);
 stringComplete = false;
   inputString = "";
    } // fin de D 

   if (inputString.substring(0,1) == "W") {        // MODIF OF KSW
    Serial.println(inputString);      
    param=inputString.substring(1);
    Serial.println(param);
    KSW=param.toInt();
setkin(FHD,FKD,FAD,MAD,KST,KSW,KML);
 stringComplete = false;
   inputString = "";
    } // fin de D 



//MODE MODIFICATION VIA SERIAL S=STANDING   B= WALKING

   // print the string when a newline arrives:
  if (stringComplete) {
    Serial.println(inputString);
    // clear the string:
    value=inputString;
    inputString = "";
    stringComplete = false;
  }
  
  
  
  if (value=="B") {
        t=t+1;
     
      if (t>100) {t=0;}
  
   myservo4.write(HD+FHD[t]);  //FLEXION HANCHE DROITE
   myservo5.write(GD-FKD[t]);  //FLEXION GENOU DROIT
   myservo6.write(CD+FAD[t]);  // DORSIFLEXION CHEVILLE DROITE
   myservo8.write(FD+MAD[t]); // Cheville plan frontal

   if (t<50) {ts=t+50;}
   if (t>50) {ts=t-50;}
   myservo1.write(HG-FHD[ts]);  // FLEXION HANCHE GAUCHE
   myservo2.write(GG+FKD[ts]);  //FLEXIN GENOU
   myservo3.write(CG-FAD[ts]);  //DORSIFLEXION CHEVILLE GAUCHE
      myservo7.write(FG-MAD[ts]); // Cheville plan frontal

   delay (T);
  }
  

  if (value=="S") {
   myservo1.write(HG);  // FLEXION LEFT HIP
   myservo2.write(GG);  //FLEXIN KNEE
   myservo3.write(CG);  //DORSIFLEXION LEFT ANKLE
   myservo4.write(HD);  //FLEXION RIGHT HIP
   myservo5.write(GD);  //FLEXION RIGHT KNEE
   myservo6.write(CD);  // DORSIFLEXION RIGHT ANKLE
   myservo7.write(int(FG));  
   myservo8.write(int(FD));  
   
   t=0;


  }

}



// SERIAL EVENT
void serialEvent() 
{
   while (Serial.available()) {
   delay(3);
   char c=Serial.read();
   inputString += c;}   
   stringComplete = true;
}

// function to recalculate the kinematics according to KST, KSW and KSF
void setkin(int FHD[],int FKD[],int FAD[],int MAD[],int KST,int KSW,int KML)
{
  
// SAGITAL PLANE  
   for (int i=0; i <= 5; i++) {
     FHD[i]=-KST;    // Hip flexion Right
     FKD[i]=0;       // Knee flexion Right
   } 
   for (int i=6; i <= 45; i++) {
     FHD[i]=KST*i/20.0-5*KST/4.0;
     FKD[i]=0;
   } 
//
   for (int i=46; i <= 55; i++) {
     FHD[i]=KST;
     FKD[i]=0;
   } 
//
   for (int i=56; i <= 80; i++) {
     FHD[i]=-KST*i/10.0+65*KST/10.0;
     FKD[i]=KSW*sin((i-55)/40.0*3.14);
   } 
//   
      for (int i=81; i <= 95; i++) {
     FHD[i]=-3*KST/2.0;
     FKD[i]=KSW*sin((i-55)/40.0*3.14);
   } 
//   
//   
         for (int i=95; i <= 100; i++) {
     FHD[i]=KST*i/10.0-11*KST;
     FKD[i]=0;
   } 
//   ANKLE 
// SAGITAL PLANE
       for (int i=0; i <= 70; i++) {
     FAD[i]=FHD[i]+FKD[i];}
     
            for (int i=71; i <= 90; i++) {
     FAD[i]=FHD[i]+1*FKD[i];}
     
     
             for (int i=91; i <= 100; i++) {
     FAD[i]=FHD[i]+1*FKD[i];}    
     
     
     
     // FRONTAL PLANE
      for (int i=0; i <= 50; i++) {
    MAD[i]= -KML*sin(2*3.14*i/100);
      }
     
           for (int i=51; i <= 75; i++) {    // petit push pendant le deuxième double support
    MAD[i]= -KML*sin(2*3.14*i/100);
      }
      
                 for (int i=76; i <= 100; i++) {
    MAD[i]= -KML*sin(2*3.14*i/100);;
      }
   
//        for (int i=0; i <= 100; i++) { 
//    Serial.println(MAD[i]);
//  }
digitalWrite(13,LOW);
delay(50);
digitalWrite(13,HIGH);
delay(50);  
digitalWrite(13,LOW);
delay(50);
digitalWrite(13,HIGH);
delay(50);  
digitalWrite(13,LOW);
delay(50);
digitalWrite(13,LOW);
delay(50);
digitalWrite(13,HIGH);
delay(50);  
digitalWrite(13,LOW);
delay(50);
digitalWrite(13,HIGH);
delay(50);  
digitalWrite(13,LOW);
delay(50);
digitalWrite(13,HIGH);
delay(50);  
digitalWrite(13,LOW);
delay(50);

analogWrite(13,60);
delay(10000);
digitalWrite(13,HIGH);
delay(70);  
digitalWrite(13,LOW);
delay(70);
digitalWrite(13,HIGH);
delay(70);  
digitalWrite(13,LOW);
delay(70);
digitalWrite(13,HIGH);
delay(70);  
digitalWrite(13,LOW);
delay(70);
digitalWrite(13,HIGH);
delay(70);  
digitalWrite(13,LOW);
delay(70);
analogWrite(13,255);
delay(5000);


}
