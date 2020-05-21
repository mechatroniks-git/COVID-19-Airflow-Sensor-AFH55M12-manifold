/*
 *
 * https://github.com/hydronics2/COVID-19-Airflow-Sensor-AFH55M12
 *
 * PCB accepts dual footprint feather ESP32 and Adafruit ItsyBitsy M0
 *
 *
 * //4/15/20 - added SLM output
 * //4/21/20 - adjusted with Patrick's curve fit calbration equation.
 * //4/21/20 - added one ms to the volume calculation
 * 4/22/20 - updated with 2.8" display and teensy4.0
 * 4/23/20 - updated new SLM and volume and pressure


*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h> // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BME280.h> //https://github.com/adafruit/Adafruit_BME280_Library

//#include <Adafruit_GFX.h>  // https://github.com/adafruit/Adafruit-GFX-Library
//#include <Adafruit_SSD1306.h>  // https://github.com/adafruit/Adafruit_SSD1306
//#include <avr/dtostrf.h> //for converting from an integer to a string for the display

#include "Display.h"
#include "SensorData.h"

Adafruit_BME280 bme1; // I2C pressure sensor under the display
long atmosphericPressure = 100167; //this bounces around a bit

//pinouts for Teensy
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 255 // RST can be set to -1 if you tie it to Arduino's reset

const int errorLED = 1; //teensy 4.0
const int userLED = 0; //teensy 4.0
const int buzzer = 23; //teensy 4.0
const int encoder0PinA = 20;  //teensy
const int encoder0PinB = 21;  //teensy
const int encoderButton = 22; //teensy
const int userButton = 5; //teensy 4.0 .. need to use pinMode(userButton, INPUT_PULLUP);
const int flowSensorPin = A0;

const float displaySeconds = 10.0f; //was 10... 
Display display;

int flowSensorValue = 0;

const int sizeBreath = 300; //was 300
int arrayBreath[sizeBreath];
int incrementBreath = 0;
int incrementGraphExtents = 0;

/*
int arraySLM[sizeBreath];
long arrayVolume[sizeBreath];
int arrayPressure[sizeBreath];
*/
SensorData slmData(
	"#1: SLM (L/min)",
	{low: 0.0f, high: 75.0f, inc: 5.0f}
);
SensorData volumeData(
	"#1: Volume (mL)",
	{low: 0.0f, high: 700.0f, inc: 50.0f}
);
SensorData pressureData(
	"#1: Pressure (cm)",
	{low: 0.0f, high: 35.0f, inc: 5.0f}
);

// Which sensor is being shown?
SensorData * sensorDatas[] = {&slmData, &volumeData, &pressureData};
const int sensorDataCount = sizeof(sensorDatas) / sizeof(sensorDatas[0]);
int showSensorIndex = 0;

boolean startBreathFlag = false;
boolean breathFlag = false;
int flowSensorThreshold = 1824; //temporary number
int averageRawSensorValue = 0;
long lastTimeSense;
int senseInterval = 20;
int updateDisplay = 40;
long lastTimeUpdateDisplay;


double timePeriod;

int pressureReading1 = 0;
int lastMaxVolume = 0;

double a1, b1, c1, d1, r2, r1, vo, tempC, tempF, tempK, ox, oy;

// Use encoder to change time range (X axis)
float timeRanges[] = {10.0f, 5.0f};
float timeIncs[] = {2.0f, 1.0f};
const int timeRangeCount = sizeof(timeRanges) / sizeof(timeRanges[0]);
int showTimeRangeIndex = 0;

// Button
int32_t buttonDownFrames = 0;

volatile int encoderClicks = 0;

#define TCAADDR 0x70
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup()  // Start of setup
{
  //while(!Serial){};
  Serial.begin(115200);
  analogReadResolution(12); //required for SAMD21 and teensy
  pinMode(flowSensorPin, INPUT);

	display.init(TFT_CS, TFT_DC);

	// Reset encoder
	attachInterrupt(digitalPinToInterrupt(encoder0PinB), encoderISR_B_HIGH, RISING);

  pinMode(errorLED, OUTPUT);
  digitalWrite(errorLED, LOW);
  pinMode(userLED, OUTPUT);
  digitalWrite(userLED, LOW);
  pinMode(buzzer, OUTPUT);
  pinMode(userButton, INPUT);
  pinMode(userButton, INPUT_PULLUP);
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  pinMode(encoderButton, INPUT);
  digitalWrite(encoderButton, HIGH); //enable pullup

  timePeriod = (double)senseInterval/1000; //time period in seconds
  Serial.print("sensing time period(seconds)= ");
  Serial.println(timePeriod);
  delay(100);  // This delay is needed to let the display to initialize

	// Attempt to find valid BME280 sensor
	while (true) {
		tcaselect(0);
		delay(10);
		if (bme1.begin(0x77, &Wire)) {
			Serial.println("Started pressure sensor BME#1");
			//break;
		}

		tcaselect(4);
		delay(10);
		if (bme1.begin(0x77, &Wire)) {
			Serial.println("Started pressure sensor BME#5");
			break;
		}

		Serial.println("Could not find one of the BME280 sensors");
		delay(3000);	// Try again in a few seconds
	}
  
  tcaselect(0); //switching back to #1
  delay(5);
  pressureReading1 = bme1.readPressure(); //ok lets take a pressure reading
  Serial.print("atmospheric pressure on #1 is: ");
  Serial.println(pressureReading1);
  atmosphericPressure = pressureReading1;

	redrawSensor();
}


void loop()  // Start of loop
{
	long now = millis();

  if(now - lastTimeSense > senseInterval)
  {
    float SLM = 0; //Standard Liters per minute (flow rate)
    flowSensorValue = analogRead(flowSensorPin);
    //Serial.println(flowSensorValue);
    lastTimeSense = now;
    if(flowSensorValue > flowSensorThreshold)
    {
      if(startBreathFlag == false)
      {
        startBreathFlag = true;

        incrementBreath = 0;
      }else
      {
        SLM = constrain(17.816955 - .029497326 * flowSensorValue + 1.231863E-5 * sq(flowSensorValue),0,100);
        //float change = (constrain(17.816955-.029497326*arrayBreath[i]+1.231863E-5*sq((float)arrayBreath[i]),0,100))*(senseInterval+1)*1000/60000; //converts SLM to mL
        Serial.print("time: ");
        Serial.print(now);
        Serial.print(" ms, SLM: ");
        Serial.print(SLM);

        float pressure = bme1.readPressure(); //ok lets take a pressure reading now that there is flow
        pressure = (pressure - atmosphericPressure)/98.07; //convert Pascals to cm of H2O
        Serial.print(", cm of H2O: ");
        Serial.print(pressure);
        //arrayPressure[incrementBreath] = (int)(pressure * 100); //package as integer
				pressureData.add(pressure, now);

        digitalWrite(userLED, HIGH);
        if(incrementBreath == 20) //debounce breaths... needs to be atleast 20 x 20ms or 400ms long
        {
          breathFlag = true;
          //displayBreath();
          digitalWrite(errorLED, HIGH);
          analogWrite(buzzer, 5);
        }

				slmData.add(SLM, now);

        float volume = volumeData.getLastValue() + SLM * (senseInterval + 1) * (1000.0f / 60000);
				volumeData.add(volume, now);
        Serial.print(", volume (mL): ");
        Serial.println(volume);
				/*
        if(incrementBreath > 0)
        {
          arrayVolume[incrementBreath] = arrayVolume[incrementBreath - 1] + SLM * 100 *(senseInterval+1)*1000/60000; //converts SLM to mL
        }else arrayVolume[0] = SLM * 100 *(senseInterval+1)*1000/60000; //converts SLM to mL
				*/

				incrementBreath++;
				if(incrementBreath == sizeBreath) { //just for safety... should never rollover
          incrementBreath = 0;
        }

      }
    }else //sensor below threshold
    {
      digitalWrite(userLED, LOW);
      digitalWrite(errorLED, LOW);
      analogWrite(buzzer, 0);
      if(breathFlag) //a breath just finished... calculate the volume
      {
        lastMaxVolume = printResults();
      }
      startBreathFlag = false;
      breathFlag = false;

			// Add zeros to all SensorData's
			for (int i = 0; i < sensorDataCount; i++) {
				sensorDatas[i]->add(0, now);
			}
    }
  }

	// Button
  if (digitalRead(userButton) == LOW) {
		//Serial.println("button Down");
		buttonDownFrames++;

		// Cheap debounce: After 2 consecutive frames,
		// button is considered pressed.
		if (buttonDownFrames == 2) {
			goToNextSensor();
		}

	} else {
		buttonDownFrames = 0;
	}

	// Encoder:
	// Debouncing an encoder digitally is hard .....
	// For now, this just counts clicks in either direction.
	if (encoderClicks > 30) {
		encoderClicks -= 30;
		showTimeRangeIndex = (showTimeRangeIndex + 1) % timeRangeCount;
		redrawSensor();
	}

//  int lastVolumeNumber = 0;
//  if(showSensorIndex == 1){
//    lastVolumeNumber = volumeData.getLastValue();
//  }

  
	// Draw graph
	SensorData * data = sensorDatas[showSensorIndex];
	display.drawGraph(data, now, lastMaxVolume);
}

// Encoder: Interrupt Service Routine
void encoderISR_B_HIGH() {
	encoderClicks++;
}

void goToNextSensor()
{
	showSensorIndex = (showSensorIndex + 1) % sensorDataCount;
	redrawSensor();
}

void redrawSensor()
{
	SensorData * data = sensorDatas[showSensorIndex];  
	float timeRange = timeRanges[showTimeRangeIndex];
	float timeInc = timeIncs[showTimeRangeIndex];

	display.setMode(data->getTitle(),
		{low: -timeRange, high: 0.0f, inc: timeInc},	// x axis
		data->getYAxis()
	);
}

int printResults()
{
  int maxVolume = volumeData.getLastValue();
  Serial.println();
  Serial.print("Volume (mL): ");
  Serial.println(maxVolume);
  return maxVolume;
}
