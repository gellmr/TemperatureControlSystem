/*
*
* ------------------------------------------------------
  Climate Control System
  ------------------------------------------------------

  Created      28 Sep 2011
  Last change  1  Oct 2011

  Specifications:

  The system should monitor the temperature of the room.

  Allow all aspects of the system to be configured using variables.

  Light different LEDs depending on the temperature.

  Sound an alarm if it is too hot.

  Implement an alarm reset system using a code sequence from button presses.

  Print out an array of information on request of a button press, including temperature in K, C, F, and system state.

  Activate a fan through a transistor to cool the system down if it gets above a certain temperature.

  Use different fan speeds depending on the temperature by pulsing the fan.

  Use a potentiometer as a calibration mechanism.

  It should only work when the lights are on, using a LDR.

  ------------------------------------------------------

  @author Michael Gell
  @version 0.1

  Semester 02/2011
  Lecture ICT106
  Project 1 (Workshops 8/9)
  Tutor   Khaled Daabaj

  ------------------------------------------------------

  Light Sensor
  
  VT90N2 LDR (Light dependent resistor) that varies
  from 10 to 40K Ohm depending on light intensity.
  
  More light intensity, less Resistance.
  
  Less light intensity, more Resistance.

* ------------------------------------------------------
* 
*/


// Digital IO Pins

const int ledPinA               = 2;

const int ledPinB               = 3;

const int ledPinC               = 4;

const int ledPinD               = 5;

const int ledPinE               = 6;

const int fanControlPin         = 7;

const int buttonSystemStatsPin  = 8;

const int speakerPin            = 9;

const int powerToTemperatureSensorPin = 10;

const int powerToLightSensorPin = 11;

const int buttonCodePadPin3     = 12;

const int buttonCodePadPin4     = 13;



// Analog Pins

const int thermistorAnalogPin  = 1;

const int calibrationAnalogPin = 3;

const int lightSenseAnalogPin  = 4;



// Variable Declarations

double c1 = 0.001346239;       // First  Steinhart-Hart coefficient

double c2 = 0.0002309426;      // Second Steinhart-Hart coefficient

double c3 = 0.000000098159;    // Third  Steinhart-Hart coefficient

int tempKelvin      = 0;                // Temperature, Kelvin

int tempCelsius     = 0;                // Temperature, Celsius

int tempFarenheit   = 0;                // Temperature, Farenheit

int autoPrintout               = 1;     // If set to 1, system status will be periodically displayed.

unsigned long statsTimerRecent = 0;     // Milliseconds. Time of most recent status printout.

unsigned long statsTimerDelay  = 1000;  // Milliseconds between automatic status printouts.

int fixedResistorOhms = 220;   // Ohms. This variable must be set according to the type of resistor that is used alongside the thermistor or light sensor.

float fltFixedResistorOhms = 0; // fixedResistorOhms as a float.

int fanState = 0;              // 1 == HIGH  0 == LOW

float temperatureRange = 10;   // Celsius.  The working temperature range.  This value allows calibration of the min / max / mid temperatures.

float maxTemperature   = 0;    // Celsius.  This variable is controlled by the calibration potentiometer. A thermistor reading equal or higher than this value will trigger the alarm. All LEDs are illuminated at the max temperature.

float midTemperature   = 0;    // Celsius.  This variable is controlled by the calibration potentiometer. Its value is halfway between the max and min temperatures. Temperatures above this value will activate the fan.

float minTemperature   = 0;    // Celsius.  This variable is controlled by the calibration potentiometer. No LEDs are illuminated at the minimum temperature. 

float systemVolts      = 5.0;  // Maximum voltage we can have with the Arduino Uno.

int alarmState = 0;            // 0 == Deactivated and Temperature ok.    1 == Alarm is happening.    2 == Deactivated and temperature still too high.

int celsiusSamples[15] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0};  // Store the 15 most recently sampled temperatures

int samplingInitialised = 0;                  // Set to 1 when we have initialised celsiusSamples.

float averageCelsius = 0;                     // The averaged value of celsius temperatures we have been sampling.

unsigned long lastTemperatureSampleTime = 0;  // ms value. Time that the most recent temperature sample was taken.

unsigned long temperatureSamplingDelay = 100; // ms value. Delay between the taking of each temperature sample.

unsigned long lastKeypadTime = 0; // ms value. Time of most recent keypad button press.

unsigned long maxKeypadSeqLife = 6000; // ms value. If no keypresses received, the current keycode sequence will be wiped.

unsigned long seqUsabilityDelay = 300; // enforced minimum delay between each acceptance of keypad presses.

unsigned int keypadSequence[4] = {0,0,0,0}; // Sequence entered in the keypad to deactivate the Alarm.

unsigned int keypadCorrectSeq[4] = {3,3,3,4}; // Keypad sequence required to deactivate the Alarm.

int currentSequenceIndex = 0;

unsigned int hertz = 523; // C5 Tone. (Cycles per second)

unsigned long fullcycleTimeUsec   = 1000000 / hertz;        // microseconds

unsigned long halfCycleTimeUsec   = fullcycleTimeUsec / 2;  // microseconds

int enableFan = 0;         //    0/1 == DISABLED / ENABLED

float fanDutyPercent = 0;  // Percent of duty cycle being used for fan. Eg, 100 percent == full power.

unsigned long fanCycleLengthMs = 300;   // Milliseconds. The complete length of a duty cycle, including low and high time.

unsigned long timeOfFanCycleBegin = 0;  // Milliseconds. The time at which the current fan duty cycle began.



// Function Prototypes

float calculateSteinharthart(float ohms);  // Calculates the temperature in degrees Kelvin

float getThermistorResistance(int pin);    // Calculates the resistance of the potentiometer (thermistor)

float celsiusToFarenheit(float c);         // Given the temperature in degrees Celsius, returns the temperature in Farenheit

float celsiusToKelvin(float c);            // Given the temperature in degrees Celsius, returns the temperature in Kelvin

int celsiusToFarenheit(int celsius);       // Given the temperature in degrees Celsius, returns the temperature in Farenheit

int celsiusToKelvin(int celsius);          // Given the temperature in degrees Celsius, returns the temperature in Kelvin

float kelvinToCelsius(float k);            // Given the temperature in degrees Kelvin, returns the temperature in Celsius

void setLEDTemperatureMeter(const float & cels); // Given Celsius, set the number of illuminated LEDs.

void runKeypadToDeactivateAlarm();         // Accepts keypad input to deactivate the alarm

void prntSequence();                       // Print the current entered keypad code sequence that the user is entering

void wipeSequence();                       // Clear the code sequence that the user has entered.

void runSpeaker();                         // Make sound with the piezo buzzer.

void runFan();                             // Run the fan

unsigned int statusPrintout(int & k, int & f, int & c);  // Display system status information (eg temperature, alarm state)

void calibrateWorkingRange();              // Adjust the working temperature range with the potentiometer.



void setup()
{
  Serial.begin(9600); // Enable serial output
  
  pinMode(ledPinA, OUTPUT);
  
  pinMode(ledPinB, OUTPUT);
  
  pinMode(ledPinC, OUTPUT);
  
  pinMode(ledPinD, OUTPUT);
  
  pinMode(ledPinE, OUTPUT);
  
  pinMode(fanControlPin, OUTPUT);
  
  pinMode(buttonSystemStatsPin, INPUT);
  
  pinMode(speakerPin, OUTPUT);
  
  pinMode(powerToTemperatureSensorPin, OUTPUT);
  
  pinMode(powerToLightSensorPin, OUTPUT);
  
  pinMode(buttonCodePadPin3, INPUT);
  
  pinMode(buttonCodePadPin4, INPUT);
  
  digitalWrite(ledPinA, LOW);
  
  digitalWrite(ledPinB, LOW);
  
  digitalWrite(ledPinC, LOW);
  
  digitalWrite(ledPinD, LOW);
  
  digitalWrite(ledPinE, LOW);
  
  digitalWrite(powerToTemperatureSensorPin, HIGH); // This will power the thermistor.
  
  digitalWrite(powerToLightSensorPin, HIGH);       // This will power the light sensor.
  
  digitalWrite(fanControlPin, HIGH);
  
  fltFixedResistorOhms = (float)fixedResistorOhms;
  
  midTemperature = minTemperature + ((float)(maxTemperature - minTemperature) / 2.0);
  
  lastKeypadTime = millis();
  
  lastTemperatureSampleTime = millis();
  
  statsTimerRecent = millis();
}



// Calculates Celsius, given degrees Kelvin

float kelvinToCelsius(float k)
{
  return k -= 273.15;
}



// Calculates Kelvin, given degrees Celsius

int celsiusToKelvin(int celsius)
{
  return celsius += 273.15;
}



// Calculates Farenheit, given degrees Celsius

float celsiusToFarenheit(float c)
{
  float fahr = (c * (float) ( (float)9.0 / (float)5.0) ) + 32.0;
  return fahr;
}



// Calculates Kelvin, given degrees Celsius

float celsiusToKelvin(float c)
{
  return c += 273.15;
}



// Calculates Farenheit, given degrees Celsius

int celsiusToFarenheit(int celsius)
{
  int fahr = (int) ( (float)celsius * (float) ( (float)9.0 / (float)5.0) ) + 32;
  return fahr;
}



// Returns the temperature in Kelvin, given the resistance in ohms.

float calculateSteinharthart(float ohms)
{
  double logRt = (double)log(ohms);
  
  double pow3LogRt = logRt * logRt * logRt;
  
  double t_Kelvin = ((double)1.0 / ((c1) + (c2 * logRt) + (c3 * pow3LogRt)));
                    
  return (float)t_Kelvin;
}



// Gets the resistance of the thermistor

float getThermistorResistance(int pin)
{
  int potValue = analogRead(pin); // Gives a value from (0 - 1023)
  
  int voltageInput = systemVolts;
  
  float volts = ( (float)(potValue + 1.0) * systemVolts ) / 1024.0;
  
  // FORMULA:    ThermistorResistance =  (FixedRes * (Vin / Vout)) - FixedRes
  
  float resistance = (float)( fixedResistorOhms * (voltageInput / volts)) - (float)fixedResistorOhms;
  
  return resistance;
}




/*
  Keypad presses are stored in the order that they were entered.

  If the user presses the keys 1,2,3,4... then the sequence will be {1,2,3,4}

  If the user presses the keys 3,3,3,4... then the sequence will be {3,3,3,4}

  If no keypresses have been entered yet, then the sequence will be {0,0,0,0}
*/

void prntSequence()
{
  Serial.print("{");
  
  for(int k = 0; k < 4; k++)
  {
    int theDigit = keypadSequence[k];
    
    if (theDigit != 0)
    {
      Serial.print(keypadSequence[k]);
    }
    else
    {
      Serial.print(" "); // Don't display zeros.
    }
    Serial.print(" ");
  }
  
  Serial.println("}");
}



void wipeSequence()
{
    Serial.println("wipeSequence()");
    
    keypadSequence[0] = 0;
    
    keypadSequence[1] = 0;
    
    keypadSequence[2] = 0;
    
    keypadSequence[3] = 0;
    
    currentSequenceIndex = 0;
}




// Accepts keypad input to deactivate the alarm.

void runKeypadToDeactivateAlarm()
{
  // millis overflow?
  
  if (millis() < lastKeypadTime)
  {
    // Overflow has occurred.
    
    lastKeypadTime = millis();
  }
  
  unsigned long currentKeypadTime = millis();
  
  // Clear the current keycode sequence if the user has not pressed any buttons for a while.
  
  if((currentKeypadTime - lastKeypadTime) > maxKeypadSeqLife)
  {
    if(keypadSequence[0] != 0)
    {
      wipeSequence();
    }
  }
  
  // Delay for usability
  
  if((currentKeypadTime - lastKeypadTime) > seqUsabilityDelay)
  {
    if (digitalRead(buttonCodePadPin3) == HIGH)
    {
      // Pressed keycode button 3
      
      lastKeypadTime = millis();
      
      keypadSequence[currentSequenceIndex] = 3;
      
      currentSequenceIndex = currentSequenceIndex + 1;
      
      prntSequence();
    }
    else if (digitalRead(buttonCodePadPin4) == HIGH)
    {
      // Pressed keycode button 4
      
      lastKeypadTime = millis();
      
      keypadSequence[currentSequenceIndex] = 4;
      
      currentSequenceIndex = currentSequenceIndex + 1;
      
      prntSequence();
    }
        
    // Entered a full sequence?
    
    int buttonsGood = 0;
    
    if (currentSequenceIndex >= 4)
    {
      for(int k = 0; k < 4; k++)
      {
        if (keypadCorrectSeq[k] == keypadSequence[k])
        {
          // This digit in the sequence is correct.
          
          buttonsGood ++;
        }
      }
      
      // All buttons good?
      
      if (buttonsGood == 4)
      {
        // Deactivate the alarm
        
        Serial.println("Alarm Deactivated");
        
        alarmState = 2;
      }
      else
      {
        // Invalid code. User must try again.
        
        Serial.println("Invalid code. User must try again.");
      }
      
      wipeSequence();
        
    }
  }
}




void runSpeaker()
{
  
  digitalWrite(speakerPin, HIGH);
  
  delayMicroseconds(halfCycleTimeUsec);
  
  digitalWrite(speakerPin, LOW);
  
  delayMicroseconds(halfCycleTimeUsec);
  
  return;
}




/*
  The user can adjust the potentiometer to change the temperature which sets off the alarm.

  This also affects the LEDs, which show the temperature, displayed within the working range.

  If the working range is 10 degrees, and the maximum temperature is 50 degrees,

  then a measured temperature of 40 degrees would illuminate NO LEDs,

  a measured temperature of 45 degrees would illuminate approximately 3 LEDs,

  and a measured temperature of 50 degrees would illuminate all 5 LEDs, and set off the alarm.
*/

void calibrateWorkingRange()
{
  
  // I will assume the desired temperature range does not exceed (0 - 50) degrees celsius.
  
  // temperatureRange remains fixed. From the pot value we determine the minimum temperature.
  
  // The maxTemperature will be (minTemperature + temperatureRange)
  
  // Eg,   31 == 21 + 10  (celsius)
  
  float rangeMax = 50;   // celsius
  
  float rangeMin = 0;    // celsius
  
  float rangeTotal = rangeMax - rangeMin; // eg (50 - 0) == 50
  
  float maxCalibValue = 1023;
  
  float calibrationPotValue = (float)analogRead(calibrationAnalogPin); // 0 - 1023
  
  float calibPercent = (calibrationPotValue / maxCalibValue) * 100.0; // Get the pot value as a percentage.
  
  float possibleShift = rangeTotal - temperatureRange; // eg 50 - 10 == 40  (the amount of available "room to move")
  
  float shift = (possibleShift / 100) * calibPercent;  // The amount of shift to apply
  
  minTemperature = rangeMin + shift;                           // Min temperature will have no LEDs lit.
  
  maxTemperature = minTemperature + temperatureRange;          // Max temperature is when the alarm kicks in, and all LEDs are illuminated.
  
  midTemperature = minTemperature + (temperatureRange / 2);    // Mid temperature is when the fan kicks in
}




void runFan()
{
  // Set the speed of the fan, depending on the temperature.
  
  //////////////////////////////////////////////////////////////////////////////////////////////////
  
  // The fan kicks in when the temperature reaches the 50 percent mark of our working range.
  
  // Eg, if our working range is 21 degrees (C) to 31 degrees (C)  then we have a working range
  
  // of 10 degrees. In such case the fan would kick in at 26 degrees (C) and would run at 1 % duty cycle.
  
  // The speed of the fan would steadily increase as the temperature reached maximum working range,
  
  // at which point the duty cycle would be 100%
  
  //////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //   50% ... 100%  of working temperature range:
  //
  //   must be mapped to:
  //
  //   1% duty cycle ... 100% duty cycle
  //
  //   This is a linear relationship, where the fan duty (d) will depend on the percent-of-working-range (t)
  //
  //   (t)    (d)
  //    50     0
  //    75     50
  //    100    100
  //
  //   d == (t - 50) * 2
  //
  //////////////////////////////////////////////////////////////////////////////////////////////////
  
  float workingRange = (float)(maxTemperature - minTemperature); // eg 10 degrees.
  
  float valueWithinRange = averageCelsius - minTemperature;      // eg 5 degrees
  
  float tPercent = (valueWithinRange / workingRange) * 100.0;    // This is (t) in the above formula.
  
  if (tPercent > 100)
  {
    tPercent = 100.0;
  }
  
  fanDutyPercent = (tPercent - 50) * 2.0; // Use the formula to get (d) ~ the fan duty.
  
  /*
  //////////////////////////////////////////////////////////////////////////////////
  // Test the fan PWM using the potentiometer.
  
  float calibrationPotValue = (float)analogRead(calibrationAnalogPin); // 0 - 1023
  
  calibrationPotValue /= 10.0; // 0 ... 102.3
  
  if (calibrationPotValue > 100.0)
  {
      calibrationPotValue = 100.0;
  }
  if (calibrationPotValue < 1.0)
  {
      calibrationPotValue = 0.0;
  }
  
  fanDutyPercent = (float)calibrationPotValue;
  //////////////////////////////////////////////////////////////////////////////////
  */
  
  unsigned long dutyTime = ((float)fanCycleLengthMs / 100.0) * fanDutyPercent; // (One percent) times (desired percent)
  
  /*
  ////////////////////////////////////////////////////////
  // DISPLAY OF PWM duty cycle for debug purposes
  if (fanState)
  {
    Serial.print("H");
  }
  else
  {
    Serial.print("_");
  }
  delay(10);
  ////////////////////////////////////////////////////////
  */
  
  // Pulses weaker than 20 percent are ineffective at powering the fan.
  if (fanDutyPercent > 20.0)
  {
    if ((millis() - timeOfFanCycleBegin) >= fanCycleLengthMs)
    {
      // Start of a new cycle.
      
      if (fanState == 0)
      {
        // Begin duty
        
        fanState = 1; // Fan state high.
        
        digitalWrite(fanControlPin, HIGH);
      }
      
      // Serial.println("C"); // DISPLAY OF PWM duty cycle for debug purposes
      
      timeOfFanCycleBegin = millis();
    }
    else
    {
      // Partway through the cycle.
      
      if ((millis() - timeOfFanCycleBegin) >= dutyTime)
      {
        if (fanState == 1)
        {
          // Done duty
          
          fanState = 0; // Fan state low.
          
          digitalWrite(fanControlPin, LOW);
        }
      }
    }
  }
  else
  {
    // Deactivate fan
    
    // Serial.println("off"); // DISPLAY OF PWM duty cycle for debug purposes
        
    fanState = 0; // Fan state low.
    
    digitalWrite(fanControlPin, LOW);
  }
}




unsigned int statusPrintout(int & k, int & f, int & c)
{
  if (!k)
  {
    return 0;
  }
  
  Serial.print(" (min ");
  
  Serial.print(minTemperature);
  
  Serial.print("  mid ");
  
  Serial.print(midTemperature);
  
  Serial.print("  max ");
  
  Serial.print(maxTemperature);
  
  Serial.print(")");
  
  
  Serial.print("   Celsius ");
  
  Serial.print(c);
  
  Serial.print("   Kelvin ");
    
  Serial.print(k);
  
  Serial.print("   Farenheit ");
  
  Serial.print(f);
  
  
  if (alarmState == 0)
  {
    Serial.print("   Temperature is ok.        Alarm OFF.        ");
  }
  else if (alarmState == 1)
  {
    Serial.print("   Temperature is too high.  Alarm ON.         ");
  }
  else if (alarmState == 2)
  {     
    Serial.print("   Temperature is too high.  Alarm OVERRIDDEN. ");
  }
  
  Serial.print("   Fan Duty ");
  
  Serial.print(fanDutyPercent);
  
  Serial.print(" %");
  
  
  Serial.print("   Averaged ");
  
  Serial.print(averageCelsius);
  
  Serial.print(" (C)");
  
  
  Serial.print(" \n");
  
  return 1;
}




/*
  Uses light sensor to detect if the lights are on in the room.

  Returns 1 if the lights are on.

  Returns 0 if the lights are NOT on.
*/
unsigned int lightsAreOn()
{
  float lightSenseValue = (float)analogRead(lightSenseAnalogPin); // 0 - 1023
  
  if (lightSenseValue >= 5)
  {
    return 1;
  }
  
  return 0;
}




void everythingOff()
{
  digitalWrite(ledPinA, LOW);
  
  digitalWrite(ledPinB, LOW);
  
  digitalWrite(ledPinC, LOW);
  
  digitalWrite(ledPinD, LOW);
  
  digitalWrite(ledPinE, LOW);
  
  digitalWrite(fanControlPin, LOW);
  
  digitalWrite(speakerPin, LOW);
}




void snooze()
{
  /*
  This function uses some code modified from the following (public domain) example code:
   
  Fading
   
  Created 1 Nov 2008
  By David A. Mellis
  Modified 17 June 2009
  By Tom Igoe
   
  http://arduino.cc/en/Tutorial/Fading       
  */
 
  // fade in from min to max in increments of 5 points:

  for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5)
  {
    analogWrite(ledPinE, fadeValue); // sets the value (range from 0 to 255):
    
    delay(10); // wait for 30 milliseconds to see the dimming effect
  }

  // fade out from max to min in increments of 5 points:

  for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=5)
  {
    analogWrite(ledPinE, fadeValue); // sets the value (range from 0 to 255):
    
    delay(10);   // wait for 30 milliseconds to see the dimming effect
  }
  
  delay(200);
}




void loop()
{
  // The system only works when the lights are on, using a LDR.
  
  if (!lightsAreOn())
  {
    // Blink an LED to show that we are sitting in the dark.    
  
    everythingOff();
    
    snooze();
    
    return;
  }
  
  calibrateWorkingRange(); // Determines the temperature that sets off the alarm.

  // millis overflow?
  
  if (millis() < lastTemperatureSampleTime)
  {
    // Overflow has occurred.
    
    lastTemperatureSampleTime = millis();
  }
  
  unsigned long currentTime = millis();

  if ((currentTime - lastTemperatureSampleTime) > temperatureSamplingDelay)
  {
    // Sample the temperature
    
    lastTemperatureSampleTime = millis();
    
    float thermistorResistance = getThermistorResistance(thermistorAnalogPin); // Get the resistance
    
    float kelv = calculateSteinharthart(thermistorResistance);                 // Gives Kelvin
    
    float celsius = kelvinToCelsius(kelv);                                     // Convert Kelvin to Celsius
    
    float fahr = celsiusToFarenheit(celsius);                                  // Convert Celsius to Farenheit
  
    tempFarenheit = (int)fahr;                                                 // Typecast to int
    
    tempKelvin = (int)kelv;                                                    // Typecast to int
    
    tempCelsius = (int)celsius;                                                // Typecast to int
    
    setLEDTemperatureMeter(celsius); // Illuminates LEDs
    
    // Calculate the temperature in celsius, averaged over the past five samples taken
    
    // (I need a more gradual rate of change so I am using averaging)
    
    // Otherwise the alarm tends to stop and start at borderline values.
    

    // Got temperature samples?
    
    if(!samplingInitialised)
    {
      // Initialise the array of temperature samples
      
      for(int i = 0; i < 15; i++)
      {
        celsiusSamples[i] = celsius;
      }
      samplingInitialised = 1;
    }
    else
    {
      // Capture a new temperature sample. Discard the oldest sample

      for(int i = 0; i < 14; i++)
      {
        // This could really be more efficient with some clever use of pointers.
        
        celsiusSamples[i] = celsiusSamples[i + 1]; // Shift values along in the array.
      }
      celsiusSamples[14] = celsius; // Capture new value.
    }
          
    // Calculate the averaged temperature from our array of samples.
    
    float celcSum = 0;
    
    for(int i = 0; i < 15; i++)
    {
      celcSum += celsiusSamples[i];
    }
    
    averageCelsius = celcSum / 15.0;
    

    // When using a non-averaged temperature value, it tends to be a bit erratic,
    
    // and the alarm can go on/off/on in a short amount of time.
    
    // Averaging some samples will hopefully reduce this (undesired) effect.
    

    // float temperatureDifference = maxTemperature - celsius;     // Raw temperature value.
    
    float temperatureDifference = maxTemperature - averageCelsius; // Here we use the averaged value instead of the raw value.
    
    if (temperatureDifference > 0)
    {
      // Temperature is less than maximum.
      
      // Everything is ok.
      
      alarmState = 0;
    }
    else
    {
      // Reached or exceeded maximum temperature.
      
      // Activate the alarm.
      
      if(alarmState == 0)
      {
        Serial.println("ALARM ACTIVATED!");
        
        alarmState = 1;
      }
    }
    
    // Do we require the fan?
    
    if (averageCelsius >= midTemperature)
    {
      enableFan = 1; // Enable the fan
    }
    else
    {
      enableFan = 0; // Disable the fan
    }
  }
  
  if (enableFan)
  {
    runFan();
  }
  
  if (alarmState == 1)
  {
    // Alarm is running
    
    runKeypadToDeactivateAlarm();
    
    runSpeaker();
  }
  
  // Automatic printout of statistics information while system is running.
      
  if (autoPrintout)
  {
    // millis() overflow?
    
    if (millis() < statsTimerRecent)
    {
      statsTimerRecent = millis();  // Deal with millis() overflow
    }
    
    // Periodically display system information.
    
    if ((millis() - statsTimerRecent) > statsTimerDelay)
    {
      if (statusPrintout(tempKelvin, tempFarenheit, tempCelsius))
      {
        statsTimerRecent = millis();
      }
    }
  }
  
  // System Statistics button
  
  // Pressing this button will mess up the nice clean tone of the speaker.
  
  if (digitalRead(buttonSystemStatsPin) == HIGH)
  {
    if (statusPrintout(tempKelvin, tempFarenheit, tempCelsius))
    {
      delay(100);
    }
  }
}




/*
  Illuminates LEDs according to the following rules:

  If temperature is less than 19 percent of the working range, no LEDs are illuminated.

  Greater than or equal to 19 percent, one LED is illuminated.

  Greater than or equal to 39 percent, two LEDS.

  Greater than or equal to 59 percent: three LEDs

  Greater than or equal to 79 percent: four LEDs

  Greater than or equal to 99 percent: five LEDs.

  The working range is the maxiumum allowable temperature (eg, 31 degrees C) 

  minus the minimum allowable temperature,eg (21 degrees C).  So, for example,

  The working range might be 10 in the scenario just described.

  In such a scenario, 26 degrees would be 50 percent of the working range and

  would illuminate three LEDs.
*/
void setLEDTemperatureMeter(const float & cels)
{
  // Figure out how many LEDs to illuminate.
  
  float relative = cels - minTemperature;
  
  float range = maxTemperature - minTemperature;
  
  float percent = (relative / range) * 100;

  if(percent >= 99)
  {
    digitalWrite(ledPinA, HIGH);
  }
  else
  {
    digitalWrite(ledPinA, LOW);
  }
  
  if(percent >= 79)
  {
    digitalWrite(ledPinB, HIGH);
  }
  else
  {
    digitalWrite(ledPinB, LOW);
  }
  
  if(percent >= 59)
  {
    digitalWrite(ledPinC, HIGH);
  }
  else
  {
    digitalWrite(ledPinC, LOW);
  }
  
  if(percent >= 39)
  {
    digitalWrite(ledPinD, HIGH);
  }
  else
  {
    digitalWrite(ledPinD, LOW);
  }
  
  if(percent >= 19)
  {
    digitalWrite(ledPinE, HIGH);
  }
  else
  {
    digitalWrite(ledPinE, LOW);
  }
}
