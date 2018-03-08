String readVal = "";
bool justRead = false;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while(Serial.available())
  {
    char tmp = Serial.read();
    readVal += tmp;
    // append char to readVal until encounter newline
    if(tmp != '\n')
      continue;
    else
      justRead = true;
  }
  // only append "This is an echo from Arduino." at end of every read
  if(justRead)
  {
    readVal += "This is an echo from Arduino.";
    Serial.print(readVal);
    readVal = "";
    justRead = false;
  }
}
