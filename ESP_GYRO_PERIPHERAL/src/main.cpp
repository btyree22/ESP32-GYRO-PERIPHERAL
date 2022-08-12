/* ===============================================================
   ------------------------ PERIPHERAL ---------------------------
   ===============================================================

   Controller MAC: 24:0A:C4:EC:0D:B4
   Peripheral MAC: 24:0A:C4:EC:A6:F0

   ** PERIPHERAL READS ACCELEROMETER DATA AND TRANSMITS TO CONTROLLER **
*/

//#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include <SPI.h>
//#include "I2Cdev.h"
#include <Wire.h>
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "esp32-hal-i2c.h"


#define I2CDEV_SERIAL_DEBUG 1

#define START_TRANSMITTING_BUTTON 15

MPU6050 ACCEL; // hardcode I2C address = 0x68 or 0x69

//uint8_t controllerBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x07, 0xCC }; // good one
//uint8_t controllerBroadcastAddress[] = { 0x30, 0xC6, 0xF7, 0x29, 0xBE, 0x68 }; // test unit
uint8_t controllerBroadcastAddress[] = { 0x24, 0x0A, 0xC4, 0xEC, 0x0D, 0xB4 };   // newest
uint8_t peripheralBroadcastAddresss[] = { 0x24, 0x0A, 0xC4, 0xEC, 0xA6, 0xF0 };

// these variables are for holding any received data
// for use with ACCEL.getMotion6(..) function
float incoming_accel_x_raw;
float incoming_accel_y_raw;
float incoming_accel_z_raw;
float incoming_gyro_x_raw;
float incoming_gyro_y_raw;
float incoming_gyro_z_raw;
// for use with ACCEL.getAcceleration(..) function
float incoming_accel_x_processed;
float incoming_accel_y_processed;
float incoming_accel_z_processed;
// for use with ACCEL.getRotation(..) function
float incoming_gyro_x_processed;
float incoming_gyro_y_processed;
float incoming_gyro_z_processed;

typedef struct struct_accel_message {
  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;
  unsigned long total_measurement_time; // total time it took for 1 measurement (milliseconds)
} struct_accel_message;

struct_accel_message incomingSensorReading; // for controller code
struct_accel_message outgoingSensorReading; // for peripheral code

esp_now_peer_info_t peerInfo;

// These variables are for helping us track the time between each measurement
unsigned long count = 0;
unsigned long start_measurement_time = 0;
unsigned long end_measurement_time   = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void readCurrentACCELEROMETERValue() {
  //Serial.println("[DEBUG] inside readCurrentAcceleromterValue()");
  
  if (!dmpReady) {
    //Serial.println("dmp ready was false");
    return;
  }

  if (ACCEL.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    //Serial.println("inside FIFO if");
    ACCEL.dmpGetQuaternion(&q, fifoBuffer);
    ACCEL.dmpGetGravity(&gravity, &q);
    ACCEL.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //Serial.print("ypr[0] = "); Serial.print(ypr[0]);
    //Serial.print(" // ypr[1] = "); Serial.print(ypr[1]);
    //Serial.print(" // ypr[2] = "); Serial.println(ypr[2]);

    outgoingSensorReading.gyro_yaw = ypr[0] * 180/M_PI;
    outgoingSensorReading.gyro_pitch = ypr[1] * 180/M_PI;
    outgoingSensorReading.gyro_roll = ypr[2] * 180/M_PI;

    //Serial.print("gyro_yaw = "); Serial.print(outgoingSensorReading.gyro_yaw);
    //Serial.print(" // gyro_pitch = "); Serial.print(outgoingSensorReading.gyro_pitch);
    //Serial.print(" // gyro_roll = "); Serial.println(outgoingSensorReading.gyro_roll);
  }
  else {
    Serial.println("FIFO IF WAS FALSE!!! WHY!!!");
  }

}

void sendCurrentACCELEROMETERValue() {
  // Send message via ESP-NOW
  end_measurement_time = millis();
  outgoingSensorReading.total_measurement_time = end_measurement_time - start_measurement_time;
  esp_err_t result = esp_now_send(controllerBroadcastAddress, (uint8_t *) &outgoingSensorReading, sizeof(outgoingSensorReading));
   
  if (result == ESP_OK) Serial.println("Sent with success");
  else Serial.println("Error sending the data");
}

void printAccelerometerDataNice() {
  // no need for peripheral to print
}

// when this microcontroller sends a message, this function is triggered
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success":"Delivery Failure");
}

// when this microcontroller receives a message, this function is triggered
void OnDataReceive(const uint8_t* mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&incomingSensorReading, incomingData, sizeof(incomingSensorReading));

  printAccelerometerDataNice();
}

void printAccelSettings() { //I'm not sure we need this section. MPU6050.cpp initialize sets clock, ranges, and sleep
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(ACCEL.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(ACCEL.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_PLL_EXT19M: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_PLL_EXT32K: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(ACCEL.getFullScaleAccelRange())
  {
    case MPU6050_ACCEL_FS_16:            Serial.println("+/- 16 g"); break;
    case MPU6050_ACCEL_FS_8:             Serial.println("+/- 8 g"); break;
    case MPU6050_ACCEL_FS_4:             Serial.println("+/- 4 g"); break;
    case MPU6050_ACCEL_FS_2:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(ACCEL.getXAccelOffset());
  Serial.print(" / ");
  Serial.print(ACCEL.getYAccelOffset());
  Serial.print(" / ");
  Serial.println(ACCEL.getZAccelOffset());
  
  Serial.println();
}

void setup() {
  // 
  pinMode(START_TRANSMITTING_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  // start I2C communication
  Wire.setPins(32, 33); // SDA SCL?
  Wire.begin();
  //Wire.begin((uint8_t)0x68, 32, 33); // address, SDA, SCL, frequency(not needed)
  Serial.print("I2C Timeout (ms): ");
  Serial.println(Wire.getTimeOut(), DEC); 

  //initialize MPU
  Serial.println("Initializing Accelerometer...");
  ACCEL.initialize();
  
  //test the connection
  Serial.println("Testing Connection...");
  while(!ACCEL.testConnection()) {
    Wire.clearWriteError();
    Serial.println("Could not find accelerometer. Check wiring?");
    delay(500);
  }

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again
  //delay(100);

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = ACCEL.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  ACCEL.setXGyroOffset(244);
  ACCEL.setYGyroOffset(-106);
  ACCEL.setZGyroOffset(-26);
  ACCEL.setZAccelOffset(783); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        ACCEL.CalibrateAccel(6);
        ACCEL.CalibrateGyro(6);
        ACCEL.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        ACCEL.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = ACCEL.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


  WiFi.mode(WIFI_STA); // station
  Serial.print("Hello, i'm peripheral, my MAC Address is: ");
  Serial.println(WiFi.macAddress());

  if(esp_now_init() != ESP_OK) {
    Serial.println("Error Initializing ESP-NOW!");
    for(;;); // just loop forever because there's no point going further
  }

  // tell the system to call the OnDataSent() function whenever data is transmitted
  esp_now_register_send_cb(OnDataSent);

  // tell this ESP who it will be talking to (the controller ESP32 which will print the Accelerometer reading out in this case)
  memcpy(peerInfo.peer_addr, controllerBroadcastAddress, 6); // ADDRESS OF THE EXPECTED RECEIVER!!
  peerInfo.channel = 0;     // default channel
  peerInfo.encrypt = false; // default encryption (none)

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    for(;;); // just lay down and die
  }

  esp_now_register_recv_cb(OnDataReceive);

// TODO: figure out how to set settings on accelerometer

  Serial.println("Entering main loop and sleeping...");
  WiFi.setSleep(true);
}

// PERIPHERAL
unsigned long last_button_press_time = 0;
unsigned long button_debounce_time = 500; // 500 milliseconds
void loop() {
  while(millis() - last_button_press_time >= button_debounce_time) {
    // read the accelerometer every X seconds and send packaged data to controller
    start_measurement_time = millis();
    readCurrentACCELEROMETERValue();
    //WiFi.setSleep(false); // try to not make the ESP32 overheat by only turning on WiFi when we need it
    sendCurrentACCELEROMETERValue();
    //WiFi.setSleep(true);

    delay(6); // this is in milliseconds. delaying the loop for 10ms *TOTAL* every iteration will 
              // yield ~100Hz measurement frequency. Sample rate is maxed at 4ms at the moment
              // NOTE: it has been tested that it takes about 4ms to take a reading from the
              // accelerometer, that's why it's delay(6) and not delay(10)
  }
  last_button_press_time = millis();
  
}