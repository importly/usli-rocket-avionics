#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>

File myFile;

// change this to match your SD shield or module;
constexpr int chipSelect = A1;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP3XX bmp;

#define SEALEVELPRESSURE_HPA (1018.96434)

[[noreturn]] void fatalError(const char* errorMessage) {
  Serial.println(errorMessage);
  while (true);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Initializing sensors...");

  pinMode(A1, OUTPUT);
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    fatalError("SD card failed.");
  }
  Serial.println("SD card ready.");

  if (!bmp.begin_I2C()) {
    fatalError("BMP388 not detected.");
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (!bno.begin()) {
    fatalError("BNO055 not detected.");
  }

  bno.setExtCrystalUse(true);

  Serial.println("Sensors ready.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("save.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to save.txt...");
    // myFile.println("  ");
    myFile.close();
    Serial.println("done.");
  } else {
    Serial.println("error opening save.txt");
  }
}




void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  // Serial.print("Temperature = ");
  // Serial.print(bmp.temperature);
  // Serial.println(" *C");

  // Serial.print("Pressure = ");
  // Serial.print(bmp.pressure / 100.0);
  // Serial.println(" hPa");

  // Serial.print("Approx. Altitude = ");
  // Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  // Serial.println(" m");

  // Serial.println();
  // delay(20);

  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  /* Display the floating point data */
  // Serial.print("X: ");
  // Serial.print(euler.x());
  // Serial.print(" Y: ");
  // Serial.print(euler.y());
  // Serial.print(" Z: ");
  // Serial.print(euler.z());
  // Serial.println("");

  delay(100);
}