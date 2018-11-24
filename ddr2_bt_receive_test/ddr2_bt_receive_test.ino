int ledPin=13;
char x;
void setup()
{
  Serial.begin(9600);
}
void loop()
{
  if(Serial.available()>0)
  {
    x=Serial.read();
    Serial.println(x);
  }
}
