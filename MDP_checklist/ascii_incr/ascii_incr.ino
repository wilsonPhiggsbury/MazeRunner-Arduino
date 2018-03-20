void setup() {
  Serial.begin(9600);

}

void loop() {
  char inchar;
  while(Serial.available()<2);
  
  inchar = Serial.read();
  Serial.read();
  char newchar = inchar+1;
  Serial.println(newchar);

}
