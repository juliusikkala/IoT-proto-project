#include <PDM.h>
#include "square_detector.h"

// This is the lowest samplerate that the PDM mic supports :/
const int samplerate = 16000;
const int tone_frequency = 440;
const int tone_period = samplerate/tone_frequency;
square_detector<tone_period, 8> detector;

void setup()
{
  Serial.begin(19200);
  while (!Serial);
  
  Serial.println("Starting init");
  pinMode(LED_BUILTIN, OUTPUT);
  PDM.onReceive(onPDMdata);
  Serial.println("Receiver set");
  if(!PDM.begin(1, samplerate))
  {
    Serial.println("PDM startup failed :(");
    while(1);
  }
  
  Serial.println("Init successful");
}


void loop()
{
  delay(100);
  Serial.println(detector.detection(), DEC);
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
