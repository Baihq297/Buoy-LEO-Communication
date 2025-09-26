#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <GY521.h>
#include <TinyGPSPlus.h>
#include "BluetoothSerial.h"
#include <Wire.h> // Tambahkan library Wire untuk GY521 (I2C)

// =================================================================
// 1. DEKLARASI OBJEK
// =================================================================
GY521 sensor(0x68);
TinyGPSPlus gps;
BluetoothSerial SerialBT;

// Define pin on ESP32
const int voltPin = 34;    // Voltage sensor
const int tempPin = 26;    // Temp and humidity sensor
const int LDRPin = 35;     // LDR sensor

// =================================================================
// 2. DEKLARASI VARIABEL GLOBAL
// =================================================================
String tanggal = "Not valid";
String waktu = "Not valid";

float accPitch;
float accRoll;
float vibrate;
float tegangan;
float suhu;
float humidity;
uint16_t cahaya;
uint32_t satelite;
double longitude;
double latitude;

float initialPitch = 0;
float initialRoll = 0;
bool isFirstRead = true;

unsigned long sensslowMillis = 0;
unsigned long senssmidMillis = 0;
unsigned long senssfstMillis = 0;

// =================================================================
// 3. PROTOTYPE FUNGSI
// =================================================================
void sensor_init();
void sensor_read();
void imu_init();
void gps_init();
void imu_read();
void volt_read();
void temp_read();
void lumen_read();
void gps_read();


// =================================================================
// 4. FUNGSI UTAMA (setup dan loop)
// =================================================================

void setup() {
  Serial.begin(250000);
  // Inisialisasi Serial2 untuk GPS pada pin 16 (RX) dan 17 (TX)
  Serial2.begin(115200, SERIAL_8N1, 16, 17); 
  SerialBT.begin("End_Node");
  while (!Serial);

  Serial.println("Data Sensor Test");
  SerialBT.println("Data Sensor Test");
  
  // Memanggil fungsi inisialisasi
  sensor_init(); 
}

void loop() {
  sensor_read();
  delay(1000); // Penundaan utama loop (dapat disesuaikan)
}


// =================================================================
// 5. DEFINISI FUNGSI PEMBANTU
// =================================================================

void sensor_init() {
  imu_init();
  gps_init();
}

/**
 * Fungsi pembaca sensor dengan mekanisme non-blocking (millis()).
 * Fungsi volt_read, temp_read, imu_read, dan lumen_read sudah diaktifkan.
 */
void sensor_read() {
  unsigned long currentMillis = millis();

  // Membaca Tegangan dan Suhu/Humidity setiap 10 ms (FAST)
  if (currentMillis - senssfstMillis >= 10) {
    senssfstMillis = currentMillis;
    volt_read();   
    temp_read();   
  }

  // Membaca GPS (Parsing NMEA) setiap 100 ms (MID)
  if (currentMillis - senssmidMillis >= 100) {
    senssmidMillis = currentMillis;  
    gps_read();
  }

  // Membaca IMU dan LDR setiap 1000 ms (SLOW)
  if (currentMillis - sensslowMillis >= 1000) {
    sensslowMillis = currentMillis;

    imu_read();    
    lumen_read();  
  }
}

void imu_init() {
  Wire.begin();
  delay(100);

  // Tunggu hingga IMU terhubung
  while (sensor.wakeup() == false) {
    delay(1000);
  }

  sensor.setAccelSensitivity(3);
  sensor.setGyroSensitivity(0);
  sensor.setThrottle(false);
}

void gps_init(){
  Serial.println(gps.libraryVersion());
  if(gps.libraryVersion() != 0){
      Serial.println("gps init SUCCESS");
  } else {
      Serial.println("gps init FAILED");
  }
  delay(100);
}

void imu_read() {
  sensor.read();
  float ax = (sensor.getAccelX() * 9.80665);
  float ay = (sensor.getAccelY() * 9.80665);
  float az = (sensor.getAccelZ() * 9.80665);

  vibrate = abs(ax) + abs(ay) + abs(az);
  float pitch = (180 * atan2(ax, sqrt(ay*ay + az*az))/PI) * (-1);
  float roll = 180 * atan2(ay, sqrt(ax*ax + az*az))/PI;

  if (isFirstRead) {
    initialPitch = pitch;
    initialRoll = roll;
    isFirstRead = false;
  }

  accPitch = pitch - initialPitch;
  accRoll = roll - initialRoll;

  Serial.print("Relative Pitch: ");
  Serial.print(accPitch); 
  Serial.println(" Derajat");

  Serial.print("Relative Roll: ");
  Serial.print(accRoll);
  Serial.println(" Derajat");
}

void volt_read() {
  static float last_tegangan;
  float alpha = 0.97;
  int tegangan_raw;
  tegangan_raw = (alpha * last_tegangan) + ((1.0 - alpha) * analogRead(voltPin)); 
  last_tegangan = tegangan_raw;
  
  // Menggunakan map untuk kalibrasi
  tegangan = map(tegangan_raw, 592, 900, 900, 1250); 
  tegangan = tegangan / 100.0;
  tegangan = tegangan < 0.0 ? 0.0 : tegangan;

  Serial.print("Tegangan: ");
  Serial.print(tegangan);
  Serial.println(" Volt");
}

void temp_read() {
  static int last_Vo;
  float alpha = 0.85;
  int Vo;
  float R1 = 10000;
  float logR2, R2, T, Dp = 23.5;
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

  Vo = (alpha * last_Vo) + ((1.0 - alpha) * analogRead(tempPin));
  last_Vo = Vo;
  R2 = R1 * (4096.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  suhu = T - 273.15 + 3.0;
  
  // Perhitungan Kelembaban (Relatif)
  humidity = 0.4 * 100 * (exp((17.625 * Dp) / (243.04 + Dp)) / exp((17.625 * suhu) / (243.04 + suhu)));

  Serial.print("Suhu: ");
  Serial.print(suhu);
  Serial.println(" Derajat Celcius");

  Serial.print("Humidity: ");
  Serial.println(humidity);
}

void lumen_read(){
  cahaya = analogRead(LDRPin);
  Serial.print("Intensitas Cahaya: ");
  Serial.println(cahaya);
}

void gps_read(){
  static unsigned long gps_milis;
  unsigned long current_millis = millis();

  // Memproses data mentah GPS
  while (Serial2.available() > 0)
    gps.encode(Serial2.read());

  if(current_millis - gps_milis >= 1000){
    gps_milis = current_millis;
    
    // Hanya cetak jika lokasi valid (ada GPS fix)
    if(gps.location.isValid()){
      longitude = gps.location.lng();
      latitude = gps.location.lat();
      tanggal = String(gps.date.day()) + "/"+ String(gps.date.month()) + "/" + String(gps.date.year());
      
      // Penyesuaian +7 jam untuk WIB
      waktu = String(gps.time.hour() + 7) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()); 
      
      Serial.print("Longitude: "); Serial.println(longitude, 6);
      SerialBT.print("Longitude: "); SerialBT.println(longitude, 6); 

      Serial.print("Latitude: "); Serial.println(latitude, 6);
      SerialBT.print("Latitude: "); SerialBT.println(latitude, 6);

      Serial.print("Tanggal: "); Serial.println(tanggal);  
      Serial.print("Waktu: "); Serial.println(waktu);
      SerialBT.print("Tanggal: "); SerialBT.println(tanggal);  
      SerialBT.print("Waktu: "); SerialBT.println(waktu);
      
    } else {
      Serial.println("Waiting for GPS fix...");
      SerialBT.println("Waiting for GPS fix...");  
    }
    
    // Bagian Satelit (opsional) - tidak dicetak
    //if(gps.satellites.isValid()){
     // satelite = gps.satellites.value();
   // }  
  }
} 