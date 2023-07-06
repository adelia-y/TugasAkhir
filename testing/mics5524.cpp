#include <Arduino.h>
#include "DFRobot_MICS.h"
using namespace std;

#define CALIBRATION_TIME 3 // Default calibration time is three minutes
#define SAMPLE_AMOUNT 1000
#define SAMPLING_TIME 1 // Sampling time in minute
#define RECENT_SAMPLE_AMOUNT 200

#define ADC_PIN 34
#define POWER_PIN 10
DFRobot_MICS_ADC mics(/*adcPin*/ADC_PIN, /*powerPin*/POWER_PIN);

// variables to generate random numbers to test algorithm without using sensor
int max_num = 100;
int min_num = 0;
int range = max_num - min_num + 1;

unsigned long sampling_delay = (SAMPLING_TIME * 60 * 1000) / SAMPLE_AMOUNT; // ms
float nh3 = 0; // ppm
float samples[SAMPLE_AMOUNT];
float recent_samples[RECENT_SAMPLE_AMOUNT];

float Q1, Q3;

float sum = 0;
float average = 0;
int count_sample = 0;

int median(float* a, int l, int r);
void IQR(float* a, int n);

void setup() {
    // Setup serial
    Serial.begin(115200);
    while(!Serial);

    // Setup sensor
    while(!mics.begin()) {
      Serial.println("No devices!");
      delay(1000);
    }
    Serial.println("Device connected successfully.");

    uint8_t mode = mics.getPowerState();
    if(mode == SLEEP_MODE) {
      mics.wakeUpMode();
      Serial.println("Wake up sensor success.");
    }
    else {
      Serial.println("The sensor is wake up mode.");
    }

    while(!mics.warmUpTime(CALIBRATION_TIME)) {
      Serial.println("Please wait until the warm-up time is over!");
      delay(1000);
    }
}

void loop() 
{
  // take raw samples
  for(int i = 0; i < SAMPLE_AMOUNT; ++i) {
    nh3 = mics.getGasData(NH3);
    // nh3 = rand() % range + min_num;
    samples[i] = nh3;
    delay(sampling_delay);
  }

  // take recent samples
  for(int i = 0; i < RECENT_SAMPLE_AMOUNT; ++i) {
    recent_samples[i] = samples[SAMPLE_AMOUNT - i];
  }
  
  // calculate outliers
  IQR(recent_samples, RECENT_SAMPLE_AMOUNT);
  
  // remove outliers
  for(int i = 0; i < RECENT_SAMPLE_AMOUNT; ++i) {
    if ((recent_samples[i] < Q1) || (recent_samples[i] > Q3)) {
      recent_samples[i] = 0;
    }
  }

  // find average
  sum = 0;
  count_sample = 0;
  for(int i = 0; i < RECENT_SAMPLE_AMOUNT; ++i) {
    sum += recent_samples[i];
    if (recent_samples[i] != 0) {
      count_sample += 1;
    }
  }
  average = sum / count_sample;
  Serial.print(average);
  Serial.println(" PPM");
  //mics.sleepMode();
}

// Function to give index of the median
int median(float* a, int l, int r) {
    int n = r - l + 1;
    n = (n + 1) / 2 - 1;
    return n + l;
}
 
// Function to calculate IQR
void IQR(float* a, int n) {
    sort(a, a + n);
 
    // Index of median of entire data
    int mid_index = median(a, 0, n);
 
    // Median of first half
    Q1 = a[median(a, 0, mid_index)];
 
    // Median of second half
    Q3 = a[mid_index + median(a, mid_index + 1, n)];
 
    return;
}