
   #include <Wire.h>
   #include <SPI.h>
   #include <Adafruit_Sensor.h>
   #include <Adafruit_BMP280.h>
   #include "Adafruit_BMP3XX.h"


  #define SEALEVELPRESSURE_HPA (1013.25)
  Adafruit_BMP280 bmp280; // object instance
  Adafruit_BMP3XX bmp390; //object instance


void setup() {
//ask to reduce redundancy of Serial begin
  start_bmp280();

  start_bmp390();
  
      
}

void loop() {
  run_bmp280();
  run_bmp390();
 
  

}


/* There are 2 types of functions, the initialization, which goes into the set-up and are exceuted ones, and the data gathering functions, ran in the loop
 * 
 */

 //for the BMP 390

void start_bmp390(){
  Serial.begin(9600); //modified and set to default for all sensors
  while (!Serial); //checking if serial excecutes
  Serial.println("Adafruit BMP390 test");

  if (!bmp390.begin_I2C()) {   //check communications
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization- set by manufacturer to filter data
  bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp390.setOutputDataRate(BMP3_ODR_50_HZ); 
  
}

void run_bmp390(){
  if (! bmp390.performReading()) { //if unable to perform reading
    Serial.println("Failed to perform reading :(");
    return;
  }
  
  Serial.print("Temperature = ");
  Serial.print(bmp390.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp390.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp390.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
}

//for the BMP 280

void start_bmp280(){
   Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp280.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp280.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);

     /* Default settings from manufacturer */
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
}

void run_bmp280(){
Serial.print(F("Temperature = "));
    Serial.print(bmp280.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp280.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp280.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
    delay(2000);


}
