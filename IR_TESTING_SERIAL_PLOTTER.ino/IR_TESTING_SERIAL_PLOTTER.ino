// void setup() {
//   // put your setup code here, to run once:

// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   Serial.println(analogRead(1));
//   Serial.println(analogRead(2));
//   Serial.println(analogRead(3));
//   Serial.println(analogRead(4));
//   Serial.println(analogRead(5));
//   delay(2000);
// }


const int numChannels = 5; // Number of channels in the IR array
const int analogPins[numChannels] = {A1, A2, A3, A4, A5}; // Analog input pins

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read data from each channel and send it to the serial port
  for (int i = 0; i < numChannels; i++) {
    int sensorValue = digitalRead(analogPins[i]);
    Serial.print(sensorValue);
    Serial.print("\t"); // Tab character to separate values
  }
  Serial.println(); // End of data for this iteration
  delay(100); // Adjust delay as needed
}
