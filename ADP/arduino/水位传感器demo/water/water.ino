/*
红线接5v,黄线为信号线(自行挑选引脚,此处用pin2),蓝线接地,黑线接地
信号线为低电平时,说明检测到水
*/
bool waterstate=LOW;
const int waterpin=2;
const int ledpin=13;
const int relaypump=12;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(waterpin,INPUT);
  pinMode(ledpin,OUTPUT);
  pinMode(relaypump,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  waterstate=digitalRead(waterpin);
  if(waterstate==LOW){
    digitalWrite(ledpin,HIGH);
    digitalWrite(relaypump,LOW);
    Serial.println("reached");
  }else{
    Serial.println("unreached");
    digitalWrite(ledpin,LOW);
    digitalWrite(relaypump,HIGH);
  }
}
