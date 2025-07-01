#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;
int a = 0;
int b = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect (only needed for some boards)
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1); // Halt execution if the card is not found
  }
  Serial.println("Card initialized.");
}

void loop() {
  String dataString = ""; 

  // Read and store the value of 'a'
  int sensor1 = a;
  dataString += String(sensor1);

  dataString += ",";     //adds a comma between values so we can use comma delimited approach to separating the values of a and b later in Excel

  // Read and store the value of 'b'
  int sensor2 = b;
  dataString += String(sensor2);

  // Open file for writing
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString); // Also print to serial monitor
  } else {
    Serial.println("error opening datalog.txt");
  }

  a = a + 1; // Increment 'a' by 1
  b = b + 1; // Increment 'b' by 1
  delay(1000); // Wait for 1 second
}

