#include <HaozeRobot.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "StepData.h"
Haoze_H1mini haoze;
//#include "PS2.h"
//Joint calibration data storage starting position
#define Servo_Addr 0

#define del 100
#define deltr 3
#define led 2               //When the LED is at a low level, the lamp illuminates; when at a high level, the lamp extinguishes.

#define wifi
//#define ps2

#ifdef ps2

#include <PS2X_lib.h>  //for v1.6

#define PS2_DAT        13  //13    
#define PS2_CMD        15  //15
#define PS2_SEL        14  //14
#define PS2_CLK        12  //12

//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

#endif

/***************************************************************************************/

#ifdef wifi

#define MAX_SRV_CLIENTS 3   //The maximum number of simultaneous connections, i.e. the number of devices you wish to connect, is limited to five for the 8266TCPServer.

const char *ssid = "Haoze"; 
const char *password = "haozerobot"; 
WiFiServer server(8266);//The port number you require may be modified at your discretion, within the range 0 to 65535.
WiFiClient serverClients[MAX_SRV_CLIENTS];
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();               //Drive servos 1 to 16 or (0 to 15)
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);          //Drive servos 17 to 32 or (16 to 31)

#define servo180
//#define servo270   //If your servo is a 270-degree servo, then uncomment this line of code and comment out the previous line.

#ifdef servo180
//#define SERVOMIN  102               //0.5/20 * 4096 = 102
//#define SERVOMID  307               //1.5/20 * 4096 = 307
//#define SERVOMAX  512               //2.5/20 * 4096 = 512

#define SERVOMIN  102               
#define SERVOMID  327               
#define SERVOMAX  552
#endif

#ifdef servo270
//#define SERVOMIN  102               //0.5/20 * 4096 = 102
//#define SERVOMID  307               //1.5/20 * 4096 = 307
//#define SERVOMAX  512               //2.5/20 * 4096 = 512

#define SERVOMIN  177           
#define SERVOMID  327               
#define SERVOMAX  577
#endif

//pwm.setPWM(i, 0, pulselen);The first parameter is the number of channels; the second is the high-level starting point, which begins at zero; the third parameter is the high-level ending point.

//a: Forward; b: Backward; c: Left turn; d: Right turn; e: Stop; f: Move sideways left; g: Move sideways right; h: Gait switch; i: Height switch; j: Squat in place; k: Jump;
//last_cmd denotes the previous command and may indicate the robot's state.
char cmd = 'e',last_cmd = 'e';
//gait denotes gait type, where 1 indicates undulating gait and 0 denotes triangular gait; body denotes height, where 0 represents the shortest, 1 denotes medium height, and 2 denotes the tallest; robotstatus denotes robot status,
int gait=0,body=0,robotstatus=0;


// Turning radius control ratio, forward and reverse speed ratio
float zhuanwan_k = 1;
float qianhou_k = 1;

//
signed char rec_lin;
signed char rec[18] = {
  0,0,0,
  0,0,0,
  0,0,0,
  0,0,0,
  0,0,0,
  0,0,0
};
// Robot's current joint variables, servo's current angle
float Servo_p[18] = {
  0.0 , 0.0 , 0.0 ,
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0
};

// Robot's current joint variables, servo's current angle
float zero_p[18] = {
  0.0 , 0.0 , 0.0 ,
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0
};
// Robot target joint variables, servo target angles
float Servo_r[18] = {
  0.0 , 0.0 , 0.0 ,
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0
};
// Servo direction inversion parameter
int direct[18] = {-1,1,1,
-1,1,1,
-1,1,1,
-1,1,1,
-1,1,1,
-1,1,1
};

int gait_zhen = 0;

void iic_device_test()//Scan the IIC chip. If it flashes once upon power-up, this indicates that 0x40 was not detected; if it flashes twice, this indicates that 0x41 was not detected.
{
  bool iic_flag[2];//Define an IIC flag array to represent the IIC scan results.
  Wire.beginTransmission(0x40);
  if(Wire.endTransmission()!=0)//0 indicates the device has been scanned; non-zero indicates the device has not been scanned.
  while(1)
  {
    digitalWrite(led,0);
    delay(100);
    digitalWrite(led,1);
    delay(1000);
  }
  Wire.beginTransmission(0x41);
  if(Wire.endTransmission()!=0)
  while(1)
  {
    digitalWrite(led,0);
    delay(100);
    digitalWrite(led,1);
    delay(100);
    digitalWrite(led,0);
    delay(100);
    digitalWrite(led,1);
    delay(1000);
  }
};

void print_jointjz(){
  Serial.println();
  Serial.println("The following is the biaoding :");
  Serial.print(" ");
  for(int i=0;i<18;i++)
  {
    Serial.print(i);
    Serial.print("    ");
  }
  Serial.println();
  for(int i=0;i<18;i++)
  {
    rec_lin = EEPROM.read(i);
    Serial.print(rec_lin);
    Serial.print("  ");
  }
  Serial.println();
}

void blink()
{
    static long previousMillis = 0;
    static int currstate = 0;
 
    if (millis() - previousMillis > 200)  //200ms
    {
        previousMillis = millis();
        currstate = 1 - currstate;
        digitalWrite(led, currstate);
    }
}

void setup() {
  // put your setup code here, to run once:
    //Define the interface numbers for each joint servo
  haoze.Jointservo[0] = 0;haoze.Jointservo[1] = 1;haoze.Jointservo[2] = 2;
  haoze.Jointservo[3] = 18;haoze.Jointservo[4] = 4;haoze.Jointservo[5] = 5;
  haoze.Jointservo[6] = 6;haoze.Jointservo[7] = 19;haoze.Jointservo[8] = 8;
  haoze.Jointservo[9] = 9;haoze.Jointservo[10] = 10;haoze.Jointservo[11] = 20;
  haoze.Jointservo[12] = 12;haoze.Jointservo[13] = 13;haoze.Jointservo[14] = 14;
  haoze.Jointservo[15] = 15;haoze.Jointservo[16] = 16;haoze.Jointservo[17] = 17;

  haoze.Jointservo[3] = 18;haoze.Jointservo[7] = 19;haoze.Jointservo[11] = 20;

  pinMode(led, OUTPUT);
  digitalWrite(led, 1);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Haoze_H1mini HexapodRobot program!");

  Wire.begin();//Initiating IIC communication
  iic_device_test();
  Serial.println("iic!");
  
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  Serial.println("PCA9685!");

  //Store the offset at 0 degrees
  EEPROM.begin(4095);// Initialise EEPROM size
  //Determine whether the EEPROM has been initialised
  rec_lin = EEPROM.read(4094);//The final one represents the initialisation flag: 127 indicates initialised, otherwise it is not initialised.
  delay(1);
  if(rec_lin != 127)
  {
    for(int i=0;i<18;i++)
    {
      EEPROM.write(Servo_Addr+i,0);
      EEPROM.commit();  
      delay(1);    
    }
    EEPROM.write(4094,127);
    EEPROM.commit();  
  }
  else;
  
  
  // Print all data from the EEPROM region
  for(int i=0;i<18;i++)
  {
    rec_lin = EEPROM.read(i);
    rec[i] = rec_lin;
    Serial.print(rec[i]);Serial.print(",");
    delay(1);
  }  
  Serial.println();

  
  for(int i=0;i<18;i++)
  Servo_p[i] = float(rec[i]);
  ESP.wdtFeed();                    
  delay(100);

  digitalWrite(led, 0);
  haoze.init();
  haoze.framewrite(zero_p);
  delay(100);
  haoze.framewrite(zero_p);
  delay(100);
  #ifdef wifi
  Serial.println();
  Serial.print("Connecting the hostspot");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
        delay(500);
        Serial.print(".");
  }
  Serial.println();
  Serial.print("The robot IP address is:");
  Serial.println(WiFi.localIP());
  server.begin();
  server.setNoDelay(true);  
  #endif


  #ifdef ps2
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
  if (pressures)
    Serial.println("true ");
  else
    Serial.println("false");
  Serial.print("rumble = ");
  if (rumble)
    Serial.println("true)");
  else
    Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
//  Serial.print(ps2x.Analog(1), HEX);
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
  case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }
   #endif

  delay(1000);

  for(int i=0;i<18;i++)
  Servo_p[i] = 0.0;

  //Transition from calibration attitude to forward preparation attitude
  haoze.framewrite(Servo_p);
  haoze.frame2frame(Servo_p,forwarda[0]);
  haoze.framewrite(forwarda[0]);  
}

void loop() {
   blink();
    #ifdef wifi
    uint8_t i;
    if (server.hasClient())
    {
        for (i = 0; i < MAX_SRV_CLIENTS; i++)
        {
            if (!serverClients[i] || !serverClients[i].connected())
            {
                if (serverClients[i]) serverClients[i].stop();
                serverClients[i] = server.available();
                continue;
            }
 
        }
        WiFiClient serverClient = server.available();
        serverClient.stop();
    }
    for (i = 0; i < MAX_SRV_CLIENTS; i++)
    {
        if (serverClients[i] && serverClients[i].connected())
        {
            digitalWrite(led, 0);//As long as a link exists, it remains permanently illuminated.
 
            if (serverClients[i].available())
            {
                while (serverClients[i].available()) 
                cmd = serverClients[i].read();
                delay(1);
                Serial.write(serverClients[i].read());
            }
        }
    }
    #endif


  #ifdef ps2
  if(error == 1) //skip loop if no controller found
    return; 
  
  if(type == 2){ //Guitar Hero Controller
    ps2x.read_gamepad();          //read controller 
   
    if(ps2x.ButtonPressed(GREEN_FRET))
      Serial.println("Green Fret Pressed");
    if(ps2x.ButtonPressed(RED_FRET))
      Serial.println("Red Fret Pressed");
    if(ps2x.ButtonPressed(YELLOW_FRET))
      Serial.println("Yellow Fret Pressed");
    if(ps2x.ButtonPressed(BLUE_FRET))
      Serial.println("Blue Fret Pressed");
    if(ps2x.ButtonPressed(ORANGE_FRET))
      Serial.println("Orange Fret Pressed"); 

    if(ps2x.ButtonPressed(STAR_POWER))
      Serial.println("Star Power Command");
    
    if(ps2x.Button(UP_STRUM))          //will be TRUE as long as button is pressed
      Serial.println("Up Strum");
    if(ps2x.Button(DOWN_STRUM))
      Serial.println("DOWN Strum");
 
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
    {
      Serial.println("Start is being held");
      cmd = 'e';
    }
      
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");
    
    if(ps2x.Button(ORANGE_FRET)) {     // print stick value IF TRUE
      Serial.print("Wammy Bar Position:");
      Serial.println(ps2x.Analog(WHAMMY_BAR), DEC); 
    } 
  }
  else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
    {
      Serial.println("Start is being held");
      cmd = 'e';
    }
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");      

    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      cmd = 'a';
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
      cmd = 'd';
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
      cmd = 'c';
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      cmd = 'b';
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }   

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if(ps2x.Button(PSB_TRIANGLE))
      {
        cmd = 'h';
        Serial.println("Triangle pressed"); 
      }
               
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
    {
      cmd = 'g';
      Serial.println("Circle just pressed");
    }
      
    if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
      {
        cmd = 'i';
        Serial.println("X just changed");
      }
      
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
    {
      cmd = 'f';
      Serial.println("Square just released"); 
    }
          

    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC); 
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC); 
    }
  }     
  #endif
  
  while(Serial.available()>0)
  {
    cmd = Serial.read();
    if(cmd=='x')//Calibrate servo angle
    {
      while(Serial.available()>0)
      {
        int servo_id=Serial.parseInt();
        haoze.rec[servo_id]=Serial.read();
        //Convert to signed char (range -128 to 127). ESP8266 defaults to unsigned char, with a range of 0 to 255.
        haoze.rec[servo_id]=Serial.parseInt();
        EEPROM.write(Servo_Addr+servo_id,haoze.rec[servo_id]);
        EEPROM.commit();
      }
      haoze.printcaliData();//Print joint calibration data
      for(int i=0;i<18;i++)
      Servo_p[i] = float(rec[i]);
      haoze.framewrite(haoze.Servo0);
      delay(100);
    }    
  }
    if(cmd == 'a')          //Forward
    {
      if(gait==0)//If it is a triangular gait
      {
        if(gait_zhen>39)
        {
          gait_zhen = 0;
        }
        if(body == 0)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda[gait_zhen][i];
        }
        else if(body == 1)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda2[gait_zhen][i];
        }
        else if(body == 2)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda3[gait_zhen][i];
        }
        
        haoze.framewrite(Servo_p);
        delay(haoze.speed);
        ESP.wdtFeed();
        gait_zhen++;
        }
      else if(gait==1)//If it is a swaying gait
      {
        if(gait_zhen>59)
        gait_zhen = 0;
      
        for(int i=0;i<18;i++)
        Servo_p[i] = forwardaF2[gait_zhen][i];
        
        haoze.framewrite(Servo_p);
        delay(haoze.speed);
        ESP.wdtFeed();
        gait_zhen++;
      }
      
    }
    else if(cmd == 'b')//Back off
    {
      if(gait==0)
      {
        if(gait_zhen<0)
        gait_zhen = 39;
        
        if(body == 0)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda[gait_zhen][i];
        }
        else if(body == 1)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda2[gait_zhen][i];
        }
        else if(body == 2)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda3[gait_zhen][i];
        }
        
        haoze.framewrite(Servo_p);
        ESP.wdtFeed();
        delay(haoze.speed);
//        delay(deltr);
        gait_zhen--;
      }
      else if(gait==1)
      {
        if(gait_zhen<0)
        gait_zhen = 59;

        for(int i=0;i<18;i++)
        Servo_p[i] = forwardaF2[gait_zhen][i];
        
        haoze.framewrite(Servo_p);
        ESP.wdtFeed();
        delay(deltr);
        gait_zhen--;
      }
    }
    else if(cmd == 'c')
    {
      if(gait==0)
      {
        if(body == 0)
        {
          for(int j=0;j<20;j++)
          {
            //The three legs on the right
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[20+j][i];

            Servo_p[15] = forwarda[20+j][15];
            Servo_p[16] = forwarda[20+j][16];
            Servo_p[17] = forwarda[20+j][17];
            
            //The three legs on the left       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed();
              delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            //right
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[j-20][i];

            Servo_p[15] = forwarda[j-20][15];
            Servo_p[16] = forwarda[j-20][16];
            Servo_p[17] = forwarda[j-20][17];
            
            //left     
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed();
              delayMicroseconds(del);
          }    
        }
        else if(body == 1)
        {
          for(int j=0;j<20;j++)
          {
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[20+j][i];

            Servo_p[15] = forwarda2[20+j][15];
            Servo_p[16] = forwarda2[20+j][16];
            Servo_p[17] = forwarda2[20+j][17];
            
                   
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed(); 
              delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[j-20][i];

            Servo_p[15] = forwarda2[j-20][15];
            Servo_p[16] = forwarda2[j-20][16];
            Servo_p[17] = forwarda2[j-20][17];
            
                  
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed(); 
              delayMicroseconds(del);
          }    
        }
        else if(body == 2)
        {
          for(int j=0;j<20;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[20+j][i];

            Servo_p[15] = forwarda3[20+j][15];
            Servo_p[16] = forwarda3[20+j][16];
            Servo_p[17] = forwarda3[20+j][17];
            
                  
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed(); 
              delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[j-20][i];

            Servo_p[15] = forwarda3[j-20][15];
            Servo_p[16] = forwarda3[j-20][16];
            Servo_p[17] = forwarda3[j-20][17];
            
                  
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed();
              delayMicroseconds(del);
          }    
        }
      }
      else if(gait==1)/
      {
          for(int j=0;j<60;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwardaF2[j][i];

            Servo_p[15] = forwardaF2[j][15];
            Servo_p[16] = forwardaF2[j][16];
            Servo_p[17] = forwardaF2[j][17];

            
            for(int i=6;i<15;i++)
            Servo_p[i] = forwardaF2[59-j][i];

            haoze.framewrite(Servo_p);
              delay(haoze.speed);
              ESP.wdtFeed();
              delay(0);
          }        
      }
    }
    else if(cmd == 'd')
    {
      if(gait==0)//If it is a triangular gait
      {
        if(body == 0)
        {
          for(int j=0;j<20;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[39-j][i];

            Servo_p[15] = forwarda[39-j][15];
            Servo_p[16] = forwarda[39-j][16];
            Servo_p[17] = forwarda[39-j][17];

            
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[20+j][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();
            delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[39-j][i];

            Servo_p[15] = forwarda[39-j][15];
            Servo_p[16] = forwarda[39-j][16];
            Servo_p[17] = forwarda[39-j][17];

            
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[j-20][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();
          }
        }
        else if(body == 1)
        {
          for(int j=0;j<20;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[39-j][i];

            Servo_p[15] = forwarda2[39-j][15];
            Servo_p[16] = forwarda2[39-j][16];
            Servo_p[17] = forwarda2[39-j][17];

           
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[20+j][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();
          } 
          for(int j=20;j<40;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[39-j][i];

            Servo_p[15] = forwarda2[39-j][15];
            Servo_p[16] = forwarda2[39-j][16];
            Servo_p[17] = forwarda2[39-j][17];

            
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[j-20][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();  
          }
        }
        else if(body == 2)
        {
          for(int j=0;j<20;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[39-j][i];

            Servo_p[15] = forwarda3[39-j][15];
            Servo_p[16] = forwarda3[39-j][16];
            Servo_p[17] = forwarda3[39-j][17];

            
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[20+j][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();              
          } 
          for(int j=20;j<40;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[39-j][i];

            Servo_p[15] = forwarda3[39-j][15];
            Servo_p[16] = forwarda3[39-j][16];
            Servo_p[17] = forwarda3[39-j][17];

            
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[j-20][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();           
          }
        }
      }
      else if(gait==1)//If it is a rippling gait
      {
          for(int j=0;j<60;j++)
          {
            
            for(int i=0;i<6;i++)
            Servo_p[i] = forwardaF2[59-j][i];

            Servo_p[15] = forwardaF2[59-j][15];
            Servo_p[16] = forwardaF2[59-j][16];
            Servo_p[17] = forwardaF2[59-j][17];
            
            
            for(int i=6;i<15;i++)
            Servo_p[i] = forwardaF2[j][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed(); 
            delay(0);
          }        
      }      
    }
    else if(cmd == 'f')//Move horizontally to the left
    {
      for(int j=0;j<60;j++)
      {
          haoze.framewrite(forwardaFH[j]);
          delay(haoze.speed);
          ESP.wdtFeed(); 
      }
    }
    else if(cmd == 'g')//right
    {
      for(int j=0;j<60;j++)
      {
          haoze.framewrite(forwardaFH[59-j]);
          delay(haoze.speed);
          ESP.wdtFeed();         
      }      
    }
    else if(cmd == 'h')//Switch gait
    {
      gait=!gait;
      cmd = 'e';
    }
    else if(cmd == 'i')//Switch height
    {
      body++;
      body=body%3;
      cmd = 'e';
    }
    else if(cmd == 'j')//Squat and stand up in place
    {
          for(int j=0;j<40;j++)
          {
              haoze.framewrite(dunqia[j]);
              delay(deltr*10);
              ESP.wdtFeed();
          }
    }
    else if(cmd == 'k')//跳跳
    {
          for(int j=0;j<40;j++)
          {
              haoze.framewrite(benga[j]);
              delay(deltr*30);
              ESP.wdtFeed();
          }
    }    
    else
    {
      delay(100);
      ESP.wdtFeed(); 
    }
}
