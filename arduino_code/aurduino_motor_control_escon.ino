// Motor Shield  
#define DIRECTION 4    //
#define ENABLE 6     //
#define SPEED A12   // 
#define SPEED_MEAS A0    //
#define CURRENT_MEAS A1    //
int count=0;
char direction1;
int speed_ref=0;
char enable;
char data[3];


void setup() {
  // Initialize all motor control pins
    Serial.begin(115200);

  pinMode(DIRECTION, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  digitalWrite(ENABLE, LOW);
  digitalWrite(DIRECTION, LOW);  
  direction1='N';
  enable='D';
  speed_ref=0;
  // Release all brakes
   analogWriteResolution(10);
  // Start all motors
  analogWrite(DAC, 0);
  data[0]=48;
  data[1]=48;
  data[2]=48;

}

void loop() {
  // Nothing needed here
    if (Serial.available() > 0) {
      int incomingByte = Serial.read();
      if (incomingByte=='E')
      {
        enable='E';        
        digitalWrite(ENABLE, HIGH);
      }
      if (incomingByte=='D')
      {
        enable='D';        
        digitalWrite(ENABLE, LOW);
      }
      if (incomingByte=='P')
      {
        direction1='P';        
        digitalWrite(DIRECTION, HIGH);
      }
      if (incomingByte=='N')
      {
        direction1='N';                
        digitalWrite(DIRECTION, LOW);
      }
      if (incomingByte=='S')
      {
        int failed=0;
        count=0;
        for (int i=0;i<3;i++)
        {
          if (Serial.available() > 0) {   
            count=0;     
            data[i]=Serial.read();
            if (data[i]<'0' || data[i]>'9')
            {
              failed=1;
              break;
            }
          }
          else
          {
            count=count+1;
            if (count>5)
            {
              failed=1;
              break;              
            }
            else
            {
              i=i-1;
              delay(1);         
            }
          }
          
        }
        if (failed==0)
        {
          speed_ref=(int) (data[0]-48.)*100+(data[1]-48.)*10+data[2]-48.;
          analogWrite(DAC, speed_ref*5.12);
          
        }
      }
    }
    int speed1=analogRead(SPEED_MEAS)-525;
    int current= analogRead(CURRENT_MEAS)-525;

    //speed1=analogRead(76);
    //current=analogRead(77);
    float speed_real=speed1*400./1024;
    float current_real=current*16./1024;
    Serial.println(String(enable)+String(direction1)+String(data[0])+String(data[1])+String(data[2])+" "+String(speed_real)+" "+String(current_real));
    delay(10);

}
