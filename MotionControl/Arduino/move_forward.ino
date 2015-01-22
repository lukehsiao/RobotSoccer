void setup() {
 Serial.begin(38400);
}
void loop() {
  //forward m1 
  Serial.write(128);
  Serial.write(0);
  Serial.write(64); // fullspeed 127, half speed 64 
  Serial.write((128+64)&(0x7F)); //change speed too
  
//    //backward m2 
//  Serial.write(128);
//  Serial.write(5);
//  Serial.write(64); // fullspeed 127, half speed 64 
//  Serial.write((128+5+64)&(0x7F)); //change speed too
//  delay(1000);
//  
  //stop m1
  Serial.write(128);
  Serial.write(0);
  Serial.write(0);
  Serial.write((128+0)&(0x7F));
  
//   //stop m2
//  Serial.write(128);
//  Serial.write(4);
//  Serial.write(0);
//  Serial.write((128+4+0)&(0x7F));
  delay(5000);
  //Serial.read();
}
