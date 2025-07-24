int ml1=2, ml2=3, mr1=7, mr2=4, enl=5, enr=6;
float x;
void setup(){
  Serial.begin(9600);
  
  pinMode(ml1,OUTPUT);
  pinMode(ml2,OUTPUT);
  pinMode(mr1,OUTPUT);
  pinMode(mr2,OUTPUT);

  digitalWrite(ml1,LOW);
  digitalWrite(ml2,LOW);
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,LOW);
  
  analogWrite(enr,100);
  analogWrite(enl,100);
}
void hold(){
  digitalWrite(ml1,LOW);
  digitalWrite(ml2,LOW);
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,LOW);
  delay(100);
}

void front(){
  digitalWrite(ml1,HIGH);
  digitalWrite(ml2,LOW);
  digitalWrite(mr1,HIGH);
  digitalWrite(mr2,LOW);
  delay(100);
}

void back(){
  digitalWrite(ml1,LOW);
  digitalWrite(ml2,HIGH);
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,HIGH);
  delay(100);
}

void left(){
  digitalWrite(ml1,LOW);
  digitalWrite(ml2,HIGH);
  digitalWrite(mr1,HIGH);
  digitalWrite(mr2,LOW);
  delay(50);
}

void right(){
  digitalWrite(ml1,HIGH);
  digitalWrite(ml2,LOW);
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,HIGH);
  delay(50);
}

void loop(){
  if(Serial.available()>0){
    x=Serial.read();
    Serial.println(char(x));
    if(x=='s'){
      hold();
    }
    if(x=='f'){
      front();
    }
    if(x=='b'){
      back();
    }
    if(x=='l'){
      left();
    }
    if(x=='r'){
      right();
    }
    if(x=='a'){
      right();
      delay(100);
      left();
      delay(100);
    }
  }
}

