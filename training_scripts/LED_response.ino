const int ldrPin = A3;    // select the input pin for the potentiometer
const int ledPin = 3;    // select the pin for the LED
int ldrValue = 0;   // variable to store the value coming from the sensor


void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}


void loop() {
  ldrValue=analogRead(ldrPin);
  Serial.println (ldrValue);   //open a serial monitor and look at the values
  if(ldrValue < 400)
  digitalWrite(ledPin,HIGH);
  else
  digitalWrite(ledPin,LOW);
  delay(500);
}

