#include <Arduino_APDS9960.h>
#include <Arduino_HTS221.h>
#include <ArduinoBLE.h>
#include <PDM.h>
#include "square_detector.h"

const char* nameOfPeripheral = "Arduino";
const char* uuidOfService = "0000181a-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxChar = "00002A58-0000-1000-8000-00805f9b34fb";

// Unfortunately I don't think its possible to send strings or any 
// other form of bulk data, would have sent rgb that way
const char* uuidOfTxCharRed = "00001140-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxCharGreen = "00001141-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxCharBlue = "00001142-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxCharAmbient = "00001143-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxCharHumidity = "00002A6F-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxCharFreq = "00001144-0000-1000-8000-00805f9b34fb";
BLEService fireService(uuidOfService);
const int RX_BUFFER_SIZE = 256;
bool RX_BUFFER_FIXED_LENGTH = false;
BLEFloatCharacteristic txChar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast); // Temperature
BLEFloatCharacteristic txCharHumidity(uuidOfTxCharHumidity, BLERead | BLENotify | BLEBroadcast); // Humidity
BLEIntCharacteristic txCharRed(uuidOfTxCharRed, BLERead | BLENotify | BLEBroadcast); // R
BLEIntCharacteristic txCharGreen(uuidOfTxCharGreen, BLERead | BLENotify | BLEBroadcast); // G
BLEIntCharacteristic txCharBlue(uuidOfTxCharBlue, BLERead | BLENotify | BLEBroadcast); // B
BLEIntCharacteristic txCharAmbient(uuidOfTxCharAmbient, BLERead | BLENotify | BLEBroadcast); // Ambient luminosity
BLEBoolCharacteristic txCharFreq(uuidOfTxCharFreq, BLERead | BLENotify | BLEBroadcast); // Frequency detection

// This is the lowest samplerate that the PDM mic supports :/
const int samplerate = 16000;
const int tone_frequency = 440;
const int tone_period = samplerate/tone_frequency;
square_detector<tone_period, 8> detector;


void startBLE() {
  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    while (1);
  }
}

// BLE connected, show light
void connectedLight() {
  digitalWrite(LED_BUILTIN, HIGH);
}

// BLE not connected, don't show light
void disconnectedLight() {
  digitalWrite(LED_BUILTIN, LOW);
}

void onBLEConnected(BLEDevice central) {
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  connectedLight();
}

void onBLEDisconnected(BLEDevice central) {
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  disconnectedLight();
}

void setup()
{
  Serial.begin(19200);
  while (!Serial);
  
  Serial.println("Starting init");
  
  PDM.onReceive(onPDMdata);
  Serial.println("Receiver set");
  if(!PDM.begin(1, samplerate))
  {
    Serial.println("PDM startup failed :(");
    while(1);
  }
  
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor.");
  }
  
  if (!HTS.begin()) {
      Serial.println("Failed to initialize humidity temperature sensor!");
  }
  
  pinMode(LED_BUILTIN, OUTPUT);
  // Start BLE.
  startBLE();

  // Create BLE service and characteristics.
  BLE.setLocalName(nameOfPeripheral);
  BLE.setAdvertisedService(fireService);
  fireService.addCharacteristic(txChar);
  fireService.addCharacteristic(txCharRed);
  fireService.addCharacteristic(txCharGreen);
  fireService.addCharacteristic(txCharBlue);
  fireService.addCharacteristic(txCharAmbient);
  fireService.addCharacteristic(txCharHumidity);
  fireService.addCharacteristic(txCharFreq);
  BLE.addService(fireService); 
   
  // Bluetooth LE connection handlers.
  BLE.setEventHandler(BLEConnected, onBLEConnected);
  BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);
  
  BLE.advertise();
  // For interested purposes, no function other than that
  // Print out full UUID and MAC address.
  Serial.println("Peripheral advertising info: ");
  Serial.print("Name: ");
  Serial.println(nameOfPeripheral);
  Serial.print("MAC: ");
  Serial.println(BLE.address());
  Serial.print("Service UUID: ");
  Serial.println(fireService.uuid());
  Serial.print("txCharacteristics UUID: ");
  Serial.println(uuidOfTxChar);
  

  Serial.println("Bluetooth device active, waiting for connections...");
}


void loop()
{
  BLEDevice central = BLE.central();
  
  if (central)
  {
    // Only send data if we are connected to a central device.
    while (central.connected()) {
      connectedLight();
      
      // If no color available, wait for a moment
      if (! APDS.colorAvailable()) {
        delay(5);
        continue;
      }
      
      int r, g, b, a;
    
      // read the color
      APDS.readColor(r, g, b, a);
   
      Serial.print("r = ");
      Serial.println(r);
      Serial.print("g = ");
      Serial.println(g);
      Serial.print("b = ");
      Serial.println(b);
      Serial.print("a = ");
      Serial.println(a);
      Serial.println();

      // Write values to the stream
      txCharRed.writeValue(r);
      txCharGreen.writeValue(g);
      txCharBlue.writeValue(b);
      txCharAmbient.writeValue(a);

      // Get and write temperature aswell
      float temperature = HTS.readTemperature();
      Serial.print("Temperature = ");
      Serial.print(temperature);
      Serial.println(" °C");
      // print an empty line
      Serial.println();
      txChar.writeValue(temperature);

      float humidity = HTS.readHumidity();
      txCharHumidity.writeValue(humidity);
      
      Serial.println(detector.detection(), DEC);
      txCharFreq.writeValue(detector.detection());
      
      // wait 1 second to print again
      delay(100);
      
      }
  } else {
    disconnectedLight();
  }
}

void onPDMdata() {
  int bytes_available = PDM.available();
  short sample_buf[512];
    
  while(bytes_available > 0)
  {   
    int count = bytes_available > sizeof(sample_buf) ? sizeof(sample_buf) : bytes_available;
    count = PDM.read(sample_buf, sizeof(sample_buf));
    bytes_available -= count;
    count /= 2;
    detector.push_buffer(sample_buf, count);
  }
}
