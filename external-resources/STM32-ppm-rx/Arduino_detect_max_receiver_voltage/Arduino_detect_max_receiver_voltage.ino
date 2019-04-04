float current_value, max_voltage;

void setup(){
  Serial.begin(57600);
}
void loop(){
  current_value = analogRead(0);
  if (max_voltage < current_value){
    max_voltage=current_value;
    Serial.println((max_voltage/1023.0)*5.0);
  }
}

