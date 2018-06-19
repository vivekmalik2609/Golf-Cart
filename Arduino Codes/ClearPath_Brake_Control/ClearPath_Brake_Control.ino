


int Step_Pin = 3;
int Dir_Pin = 5;
int Feedback_Pin = A0;
int Pos_des;
int Pos_des_given;00
byte Pos_des1; byte Pos_des2; byte Pos_des3; byte Pos_des4; byte Pos_des5; 
int margin = 20;



void setup() 
{
Serial.begin(9600);
pinMode(Step_Pin, OUTPUT);
pinMode(Dir_Pin, OUTPUT);
pinMode(Feedback_Pin, INPUT);
//  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
//  TCCR2B = _BV(CS22);
//  OCR2A = 180;
//  OCR2B = 50;

}

void loop() 
{

    if (Serial.available() > 0) {
    // read the incoming byte:
    Pos_des1 = Serial.read()-'0';
    delay(1);
    Pos_des2 = Serial.read()-'0';
    delay(1);
    Pos_des3 = Serial.read()-'0';
    delay(1);
    Pos_des4 = Serial.read()-'0';
    delay(1);
    Pos_des5 = Serial.read();
    
    Pos_des_given = (Pos_des1*1000)+(Pos_des2*100)+(Pos_des3*10)+Pos_des4;
  
    // say what you got:
    Serial.println(Pos_des1);
    Serial.println(Pos_des2);
    Serial.println(Pos_des3);
    Serial.println(Pos_des4);
    Serial.println(Pos_des);
  }

Pos_des = map(Pos_des_given,0,1024,1023,846);
int Pos_act=analogRead(Feedback_Pin);

  
int Pos_des_high = Pos_des + margin;
int Pos_des_low = Pos_des - margin;
  
if (Pos_des_low<Pos_act && Pos_act<Pos_des_high)
  {
    digitalWrite(Dir_Pin,LOW);
    analogWrite(Step_Pin,0);
    
      Serial.print(Pos_des);
    Serial.print("   ");
     Serial.print(Pos_act);
    Serial.print("   ");
    Serial.print("Stable");
    Serial.println("   ");
    return;
  }

if (Pos_des<Pos_act)
  {
    digitalWrite(Dir_Pin,LOW);
    analogWrite(Step_Pin,127);
    
    Serial.print(Pos_des);
    Serial.print("   ");
    Serial.print(Pos_act);
    Serial.print("   ");
    Serial.print("LOW");
    Serial.println("   ");
    return;
  }
if (Pos_des>Pos_act)
 {
    
    digitalWrite(Dir_Pin,HIGH);
    analogWrite(Step_Pin,127);
   
      Serial.print(Pos_des);
    Serial.print("   ");
     Serial.print(Pos_act);
    Serial.print("   ");
    Serial.print("HIGH");
     Serial.println("   ");
    return;
  }

}
