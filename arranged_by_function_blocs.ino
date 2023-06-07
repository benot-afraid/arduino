#include <Adafruit_HMC5883_U.h>       
#include <RTClib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

#define DEG_TO_RAD 0.01745329
#define PI 3.141592654
#define TWOPI 6.28318531 //2*PI

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);    //**//

const int IN1 = 7;
const int IN2 = 11;
const int IN3 = 5;
const int IN4 = 4;
const int ENA = 10;
const int ENB = 9;

float Lon =10.75; /*Monastir*/
float Lat =35.74;
float nbrdays[12] = {0.0000,28.0000,31.0000,30.0000,31.0000,30.0000,31.0000,31.0000,30.0000,31.0000,30.0000,31.0000};
char buf1[20];//for terminal dispaly
float tz=1.0000000;//time zone
float resultat[4];
      float a1;
      float k;
      float annee,mois;
      float jdoy;
      float tutc;
      float julian;
      float result_julian[2];
           float mnlong,mnanom,eclong,obleq,delta,gmst,lmst,lon,ra,ha,alpha,alphaa,alpha0d,r1,r,alpha0;
           float a,b,y,z,declination,altitude,azimuth,zenith;
           float mnlong_sans_mode;
           float heading[2];

RTC_DS3231 rtc;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);//baud rate
  Wire.begin();
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
    
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    //while (1) delay(10);
    }
if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
  /*if(timeStatus() != timeSet) 
    Serial.println("System Time Cannot be Set. Check Connections.");
  else
    Serial.println("System Time is Set.");
  delay(2000);*/
  Serial.println("Longitude and latitude: "); Serial.print(Lon);
  Serial.print(" "); Serial.println(Lat);
  //delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  sensors_event_t event; 
  mag.getEvent(&event);
  sensor_t sensor;
  mag.getSensor(&sensor);
  
  DateTime now = rtc.now();
  sprintf(buf1, "%02d:%02d:%02d %02d/%02d/%02d",  now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());  
  Serial.print(F("Date/Time: "));
  Serial.println(buf1);
                                                             //julian
  if (fmod(now.year(),4.0000) == 0.0000){
    k=1.0000;}
  else{k=0.0000;}
  if (now.month() > 1){
    for (int i=0; i<(now.month()); i++){
    a1=a1+nbrdays[i];
    }
   }
//a1=243.0000;
   jdoy=now.day()+a1;
  if (now.month() >= 3){
    jdoy=jdoy+k;
  }
  float(tutc=now.hour()+(now.minute()/60.0000)-tz,7);
  float(julian=32916.5000+365.0000*(now.year()-1949.0)+int((now.year()-1949.0)/4.0)+jdoy+tutc/24.0-51545.0,7);
     Serial.print("k= ");Serial.println(k);
     Serial.print("a=");Serial.println(a1,7);
     Serial.print("jdoy= ");Serial.println(jdoy,7);
     Serial.print("tutc= ");Serial.println(tutc,7);
     Serial.print("julian= ");Serial.println(julian,7);Serial.println("--------------");
  result_julian[0]=julian;
  result_julian[1]=tutc;
//  julian=result_julian[0];
//  tutc=result_julian[1];
  delay(1500);                                                            //sun pos
  float(mnlong_sans_mode=280.4600+0.9856474*julian,7);
   Serial.print("mnlong_sans_mode");
   Serial.println(mnlong_sans_mode,7);
   float(mnlong=fmod((280.4600+0.9856474*julian),360.0000),7);
   Serial.print("mnlong inside sun_position= ");Serial.println(mnlong,7);Serial.println("--------------");
   float(mnanom= fmod((DEG_TO_RAD*(357.5280+0.9856003*julian)),TWOPI),7);
   Serial.print("mnanom inside sun_position= ");Serial.println(mnanom,7);
   Serial.println("--------------");
   float(eclong=fmod(DEG_TO_RAD*(mnlong+1.9150*sin(mnanom)+0.0200*sin(2.0000*mnanom)),TWOPI),7);
   Serial.print("eclong inside sun_position= ");Serial.println(eclong,7);Serial.println("--------------");
   float(obleq=DEG_TO_RAD*(23.4390-0.0000004*julian),7);
   Serial.print("obleq inside sun_position= ");Serial.println(obleq,7);Serial.println("--------------");
   if (cos(eclong)<0){
    float(ra=atan((cos(obleq)*sin(eclong))/cos(eclong))+PI,7);Serial.println("cos(eclong)<0");
   }
   if (cos(obleq)*sin(eclong)<0){
    float(ra=atan((cos(obleq)*sin(eclong))/cos(eclong))+TWOPI,7); /*2 coditions if vérifiées??*/Serial.println("cos(eclong)*sin(eclong)<0");
    
   }
   Serial.print("ra= ");Serial.println(ra,7);Serial.println("--------------");
   float(delta=asin(sin(obleq)*sin(eclong)),7); //solar declination
   Serial.print("delta= ");Serial.println(delta,7);Serial.println("--------------");
   float(gmst=fmod((6.697375+0.0657098242*julian+tutc),24.0000),7);
   Serial.print("gmst= ");Serial.println(gmst,7);Serial.println("--------------");
   float(lmst=gmst+Lon/15.0000,7);Serial.println("--------------");
   Serial.print("lmst= ");Serial.println(lmst,7);Serial.println("--------------");
   float(ha=fmod(15.0000*DEG_TO_RAD*lmst-ra,3.141592654),7);
   Serial.print("ha= ");Serial.println(ha,7);Serial.println("--------------");
   
   float(alphaa=sin(delta)*sin(DEG_TO_RAD*Lon)+cos(delta)*cos(DEG_TO_RAD*Lat)*cos(ha),7);//variable for alpha0
   Serial.print("alpha a= ");Serial.println(alphaa,7);
   if ((alphaa<=1)&(alphaa>=-1)){
    float(alpha0=asin(alphaa),7);Serial.println("(alphaa<=1)&(alphaa>=-1)");
    }
   if (alphaa>1.0000){
   float(alpha0=PI/2.0000,7);Serial.println("alphaa>1.0000");
   }
    if (alphaa<-1) {
    float(alpha0=-PI/2.0000,7);Serial.println("alphaa<-1.0000");
    }
    Serial.print("alpha0= ");Serial.println(alpha0,7);
    float(alpha0d=(1/DEG_TO_RAD)*alpha0,10);Serial.print("alpha0d= ");Serial.println(alpha0d,10);Serial.println("--------------");
    if (alpha0d>-0.5600){
      float(r1=(3.51561*(0.1594+0.0196*alpha0d+0.00002*pow(alpha0d,2)))*pow(1.0000+0.5050*alpha0d+0.0845*pow(alpha0d,2.0),-1.0000),7);
      }
      else{float(r1=0.5600,7);}
      Serial.print("r1= ");Serial.println(r1,7);
    if ((alpha0d+r1)>90.0){
      float(alpha=PI/2.0,7);}
      else{float(alpha=DEG_TO_RAD*(alpha0d+r1),7);}
      Serial.print("alpha= ");Serial.println(alpha,7);Serial.println("--------------");
    float(a=(sin(alpha0)*sin(DEG_TO_RAD*Lat)-sin(delta))/(cos(alpha0)*cos(DEG_TO_RAD*Lat)),7);
    Serial.print("a= ");Serial.println(a,4);
    if ((-1.0<=a)&(a<=1.0)){
      float(b=acos(a),7);
      }
      Serial.print("(-1.0<=a)&(a<=1.0))---- b= ");Serial.println(b,4);
      if (a>1){
        b=0; Serial.print("(a>1)---- b= ");Serial.println(b,4); } 
       float(y=b+PI,7);
       if (ha<-PI){
        float(y=b,7);}
       if ((ha>=-PI)&(ha<=0.0)){
       float(y=PI-b,7);}
        Serial.print("y= ");Serial.println(y,7);
       z=float((3.141592654/2)-alpha);
       Serial.print("z= ");Serial.println(z,7);
       float(declination=delta/DEG_TO_RAD,7);
       float(altitude=alpha/DEG_TO_RAD,7);
       float(azimuth=y/DEG_TO_RAD,7);
       float(zenith=z/DEG_TO_RAD,7);
       float(resultat[0]=declination,7);
       float(resultat[1]=altitude,7);
       float(resultat[2]=azimuth,7);
       float(resultat[3]=zenith,7);
  Serial.print("declination: ");Serial.println(declination,7);
  Serial.print("altitude: ");Serial.println(altitude,7);
  Serial.print("azimuth: ");Serial.println(azimuth,7);
  Serial.print("zenith: ");Serial.println(zenith,7);
  Serial.println("---------------------");
  delay(1000);

  TrackerPosition(event);
  
  analogWrite(ENA, 250);
  analogWrite(ENB, 250);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);  
}

int TrackerPosition(sensors_event_t event) // obtenir la position du suiveur par rapport au nord
    {
     float heading[2];
     float track_elev = atan2(event.magnetic.y, event.magnetic.z);/*elevation
                                                                NOTE: This calibration method wasn’t very accurate inside my workshop (it’s all steel construction), but was fine everywhere else I tried.
                                                                So keep an eye out for things that might throw your magnetic readings out of whack.*/
     float track_az = atan2(event.magnetic.x, event.magnetic.y);/*azimuth*/
                                                                //set declinationAngle based on magnetic field strength in locality.
     float declinationAngle = 0.049;                            // declination de nord magentique par rapport au nord Geoterrestre
     heading[1]=track_az+declinationAngle;
      
      // Correct for when signs are reversed.
      if(heading[1] < 0)
        heading[1] += 2*PI;
        
      // Check for wrap due to addition of declination.
      if(heading[1] > 2*PI)
        heading[1] -= 2*PI;
    
      // Convert radians to degrees for readability.
      int headingDegrees1 = int(round(degrees(track_elev)));
      int headingDegrees2 = int(round(degrees(track_az)));
      heading[0]=headingDegrees1;
      heading[1]=headingDegrees2;
      
    }
void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
