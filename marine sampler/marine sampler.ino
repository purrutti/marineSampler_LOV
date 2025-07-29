/*
 ********************************************************
    Marine Sampler Timer 2
    for Arduino UNO
    by Paul Waller, CSL, Uni of Tasmania
    Version 3.2 (Change also at line 87)
    Date: 17/11/2019
 ********************************************************
  This timer once started will run a motor for a settable number of seconds
  repeating a settable number of times with a settable number of seconds
  between motor runs(samples). There is also an initial period of rest when
  no motor runs occurr. After the user entered period of operation is
  completed the timer goes into SLEEP mode and draws low current until the
  RESET button is pressed.
  The timer monitors the battery voltage and records the voltage after the
  run along with the number of samples
  This version allows the Adafruit DataLogger Shield version C to be used
  with the PCF8523 RTC







*/
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "RTClib.h"
#include <avr/sleep.h>
//
// constants won't change. Used here to
// set pin numbers:
const int ledPin = 6;  // the number of the LED pin
const int motorPin = 9;

const int motorCurrent = A2; // Measures Motor Current
int startButton = 7;     // pin used for initiating timer, connected to START button and RxData
// pulled LOW with button or Serial RxData line.
int battVoltPin = A0;  // pin connected to Battery via resistive divider

byte motorSpeed = 0;
volatile unsigned long elapsedTimeSecs = 0;
unsigned long missionTimeSecs = 0;
unsigned long startDelayHours = 0;
unsigned long previousMillis = 0;
int flashDuration = 100;
unsigned long O2Interval = 0;
unsigned long rotationInterval = 0;
unsigned long rotationDuration = 0;
unsigned long missionDuration = 0;
unsigned long missionDurationSecs = 0;
unsigned long collectionDurationSecs = 0;
unsigned long collectionDuration = 0;
unsigned long O2SampleCount = 1;
unsigned long rotationCount = 1;
int badRotationCount = 0;
byte logFlag = 0;
byte motorOnFlag = 0;
int errorState = 0;
unsigned long motorCurrentMeasured = 0;
unsigned long motorCurrentMeasuredLast = 0;
const int chipSelect = 10; // for the data logging shield, we use digital pin 10 for the SD cs line
SoftwareSerial mySerial(3,2); // Software Serial Ports Rx, Tx
//RTC_PCF8523 RTC; // define the Real Time Clock object
//RTC_DS1307 RTC; // define the Real Time Clock object
RTC_MCP79410 RTC;

File logfile; //The logging file
File root; //

DateTime now, previousSecond;
void setup()
{
    // set the digital pin as output:
    pinMode(startButton, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
    pinMode(motorPin, OUTPUT);
    pinMode(chipSelect, OUTPUT); // ss pin of SD Card
    Serial.begin(115200);
    mySerial.begin(9600);

    if (!RTC.begin()) {
        Serial.println("Couldn't find RTC");
    }
    if (!RTC.isrunning()) {
        Serial.println("RTC is NOT running, setting date/time!");
        // following line sets the RTC to the date & time this sketch was compiled
        //                    Y   M  D   H   M   S    enable battery backup
        RTC.adjust(DateTime(F(__DATE__), F(__TIME__)), MCP7941X_BATT_BKUP_EN);
        // To set the RTC with an explicit date & time, see below
        // For May 31, 2016 at 1:23:16 : 
        //                      Y   M  D   H   M   S    enable battery backup
        //rtc.adjust(DateTime(2016, 5, 31, 1, 23, 16), MCP7941X_BATT_BKUP_EN);
    }
    else { Serial.println("RTC already running... no time adjust"); }

    now = RTC.now();


    //RTC.writeSqwPinMode(PCF8523_SquareWave1HZ); //Set the PCF8523 SQW pin to 1Hz
    //RTC.writeSqwPinMode(SquareWave1HZ); //Set the 1307 SQW pin to 1Hz
    digitalWrite(ledPin, LOW);
    analogWrite(motorPin, 0);
   // Serial.println(F("***** Marine_Sampler_Timer2 Version 3.2 written by Paul Waller, CSL, University of Tasmania (paul.waller@utas.edu.au) *****"));
    motorSpeed = EEPROM.read(0);
    //rotateBallOnce();



    motorSpeed = EEPROM.read(0);
    Serial.print(F("Motor Speed = "));
    Serial.println(motorSpeed, DEC);
    Serial.print(F("Current Date and Time: "));
    serialDateTimeDisplay();


    startDelayHours = EEPROM.read(1);
    Serial.println();
    Serial.print(F("Current Start Delay (Hours) = "));
    Serial.println(startDelayHours);



    O2Interval = EEPROM.read(5);
    Serial.print(F("O2 Sampling Interval (Minutes)= "));
    Serial.println(O2Interval);

    rotationInterval = EEPROM.read(9);
    Serial.println();
    Serial.print(F("Current Rotation Interval is "));
    Serial.println(rotationInterval);

    rotationDuration = EEPROM.read(13);
    Serial.print(F("Rotation Duration (Seconds)= "));
    Serial.println(rotationDuration);

    missionDuration = EEPROM.read(17);
    Serial.println();
    Serial.print(F("Mission Duration is "));
    Serial.println(missionDuration);

    delay(1000);
    collectionDuration = EEPROM.read(21);
    Serial.println();
    Serial.print(F("Collection Duration is "));
    Serial.println(collectionDuration);

    readBattV();
    initOptode();
    initSDCard();
    delay(1000);

    Serial.println(F("Enter config? <y/n>"));
    serialClearAndWait();

    if (Serial.read() == 121)// Is it "y"?
    {
        Serial.println(F("Send a custom command? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121)// Is it "y"?
        {
            while (1) {
                Serial.println(F("Type the command and hit <return> or type <exit> to exit command mode"));
                serialClearAndWait();
                String s = Serial.readString();
                if (s.indexOf("exit")>=0) break;
                if (s.length() > 0) sendCmd(s);
                else break;
            }

        }


        Serial.print(F("Current Motor Speed is "));
        Serial.println(motorSpeed);
        Serial.println(F("Set Motor Speed? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121)// Is it "y"?
        {
            Serial.println(F("Enter Motor Speed (0 to 255) then hit <return> or just hit <return> to skip"));
            serialClearAndWait();
            motorSpeed = Serial.parseInt();    // read the incoming byte and convert ascii to binary:
            motorSpeed = constrain(motorSpeed, 0, 255); //constrain to pwm range for motor
            EEPROM.write(0, motorSpeed);
        }
        motorSpeed = EEPROM.read(0);
        Serial.print(F("Motor Speed = "));
        Serial.println(motorSpeed, DEC);
        EEPROM.write(0, motorSpeed);
        Serial.println();
        Serial.print(F("Current Date and Time: "));
        serialDateTimeDisplay();Serial.println(F("Set Date and Time? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121)
        {
            setRTC();
        }
        startDelayHours = EEPROM.read(1);
        Serial.println();
        Serial.print(F("Current Start Delay (Hours) is "));
        Serial.println(startDelayHours);       Serial.println(F("Enter Start Delay (Hours)? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121)
        {
            serialClearAndWait();
            Serial.println(F("Enter Start Delay (0-255 Hours) then hit <return>"));
            serialClearAndWait();
            startDelayHours = Serial.parseInt();
            startDelayHours = constrain(startDelayHours, 0, 255); //constrain to max range for start delay
            EEPROM.write(1, startDelayHours);
        }
        startDelayHours = EEPROM.read(1);
        Serial.print(F("Start Delay Hours = "));
        Serial.println(startDelayHours);
        O2Interval = EEPROM.read(5);
        Serial.println();
        Serial.print(F("Current O2 Sampling Interval is "));
        Serial.println(O2Interval);
        Serial.println(F("Enter O2 Sampling Interval (minutes)? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121) //Is it a 'y'?
        {
            serialClearAndWait();
            Serial.println(F("Enter O2 Sampling Interval (0 - 255 minutes) then hit <return>"));
            serialClearAndWait();
            O2Interval = Serial.parseInt();
            O2Interval = constrain(O2Interval, 0, 3600); //constrain to max range for O2 Sampling Interval
            EEPROM.write(5, O2Interval);
        }
        O2Interval = EEPROM.read(5);
        Serial.print(F("O2 Sampling Interval (Minutes)= "));
        Serial.println(O2Interval);
        rotationInterval = EEPROM.read(9);
        Serial.println();
        Serial.print(F("Current Rotation Interval is "));
        Serial.println(rotationInterval);
        Serial.println(F("Enter Rotation Interval (Minutes)? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121) //Is it a 'y'?
        {
            serialClearAndWait();
            Serial.println(F("Enter Rotation Interval (0-255 Minutes) then hit <return>"));
            serialClearAndWait();
            rotationInterval = Serial.parseInt();
            rotationInterval = constrain(rotationInterval, 0, 255); //constrain to max range for Rotation Interval
            EEPROM.write(9, rotationInterval);
        }
        rotationInterval = EEPROM.read(9);
        Serial.print(F("Rotation Interval (Minutes)= "));
        Serial.println(rotationInterval);
        rotationDuration = EEPROM.read(13);
        Serial.println();
        Serial.print(F("Current Rotation Duration is "));
        Serial.println(rotationDuration);
        Serial.println(F("Enter Rotation Duration (Seconds)? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121) //Is it a 'y'?
        {
            serialClearAndWait();
            Serial.println(F("Enter Rotation Duration (0-255 Seconds) then hit <return>"));
            serialClearAndWait();
            rotationDuration = Serial.parseInt();
            rotationDuration = constrain(rotationDuration, 0, 255); //constrain to max range for O2 Sampling Interval
            EEPROM.write(13, rotationDuration);
        }
        rotationDuration = EEPROM.read(13);
        Serial.print(F("Rotation Duration (Seconds)= "));
        Serial.println(rotationDuration);
        missionDuration = EEPROM.read(17);
        Serial.println();
        Serial.print(F("Mission Duration is "));
        Serial.println(missionDuration);
        Serial.println(F("Enter Mission Duration (Hours)? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121) //Is it a 'y'?
        {
            serialClearAndWait();
            Serial.println(F("Enter Mission Duration (0-255 Hours) then hit <return>"));
            serialClearAndWait();
            missionDuration = Serial.parseInt();
            missionDuration = constrain(missionDuration, 0, 255); //constrain to max range for Mission Duration
            EEPROM.write(17, missionDuration);
        }
        missionDuration = EEPROM.read(17);
        Serial.print(F("Mission Duration (Hours)= "));
        Serial.println(missionDuration);
        collectionDuration = EEPROM.read(21);
        Serial.println();
        Serial.print(F("Collection Duration is "));
        Serial.println(collectionDuration);
        Serial.println(F("Enter Collection Duration (Hours)? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121) //Is it a 'y'?
        {
            serialClearAndWait();
            Serial.println(F("Enter Collection Duration (0-255 Hours) then hit <return>"));
            serialClearAndWait();
            collectionDuration = Serial.parseInt();
            collectionDuration = constrain(collectionDuration, 0, 255); //constrain to max range for Mission Duration
            EEPROM.write(21, collectionDuration);
        }
        collectionDuration = EEPROM.read(21);
        Serial.print(F("Collection Duration (Hours)= "));
        Serial.println(collectionDuration);
       
        //  File root;
        Serial.println();
        Serial.println(F("Download File ? <y/n>"));
        serialClearAndWait();
        if (Serial.read() == 121) //Is it a 'y'?
        {
            Serial.println(F("Files on SD Card      Size"));
            root = SD.open("/");
            printDirectory(root, 0);
            Serial.println(F("Enter XX part of Filename <LOGGERXX.CSV>"));
            serialClearAndWait();
            errorState = readDataFromSD();
            while (errorState == 1)
            {
                Serial.println(F("try again..."));
                errorState = readDataFromSD();
            }
        }
    }


    Serial.println(F("Waiting for START button press (or press any key)"));
    while (digitalRead(startButton) == 1)
    {
    }
    openLogFile(); // open logfile and save mission parameters
    logfile.print(F("Motor Speed = "));
    logfile.print(motorSpeed);
    logfile.print(F(","));
    logfile.print(F("Start Delay = "));
    logfile.print(startDelayHours);
    logfile.print(F(" Hours,"));
    logfile.print(F("O2 Sampling Interval = "));
    logfile.print(O2Interval);
    logfile.print(F(" mins,"));
    logfile.print(F("Rotation Interval = "));
    logfile.print(rotationInterval);
    logfile.print(F(" mins,"));
    logfile.print(F("Rotation Duration = "));
    logfile.print(rotationDuration);
    logfile.println(F(" secs"));
    logfile.flush();
    elapsedTimeSecs = 0;
    missionDurationSecs = (missionDuration * 3600);
    collectionDurationSecs = (collectionDuration * 3600);
    logFlag = 1;
    now = RTC.now();
}
unsigned long m;
void loop()
{
    unsigned long m = millis();

    previousSecond = now;
    now = RTC.now();

    unsigned long diff = now.unixtime() - previousSecond.unixtime();
    if (m >= (previousMillis + flashDuration)) //Check each time around loop if status LED needs to be turned OFF
    {
        digitalWrite(ledPin, LOW);
    }
    //if (m >= (previousMillis + 1000)) //If an Interrupt (1 second pulse from RTC)occurred check times
    if (diff>0)
    {
        digitalWrite(ledPin, HIGH); //Turn on Status LED every second
        previousMillis = m;
        //serialDateTimeDisplay(); //Display time and Date then reset the IRQ Flag
        elapsedTimeSecs +=diff; //Update elapsed time counter

        if (elapsedTimeSecs < (startDelayHours * 3600)) //Wait for Start delay before mission
        {
            Serial.print(F("Elapsed Time (Start Delay)= "));
            Serial.print(elapsedTimeSecs);
            Serial.println(F(" seconds"));
            Serial.print(F("Time to mission start = "));
            Serial.print((startDelayHours * 3600) - elapsedTimeSecs);
            Serial.println(F(" seconds"));
            flashDuration = 20;
        }
        Serial.print(F("Mission Duration Secs = "));
        Serial.println(missionDurationSecs);
        Serial.print(F("Collection Duration Secs = "));
        Serial.println(collectionDurationSecs);
        Serial.print(F("Last Rotation Average Current (units not amps) = "));
        Serial.println(motorCurrentMeasuredLast);
        if (elapsedTimeSecs >= (startDelayHours * 3600) && elapsedTimeSecs < missionDurationSecs)
        {
            Serial.print(F("Elapsed Time (Running Mission)= "));
            Serial.print(elapsedTimeSecs);
            Serial.println(F(" seconds"));
            missionTimeSecs = elapsedTimeSecs - (startDelayHours * 3600);
            Serial.print(F("Mission Time = "));
            Serial.print(missionTimeSecs);
            Serial.println(F(" seconds"));
            Serial.print(F("O2 SampleCount = "));
            Serial.print(O2SampleCount);
            Serial.println(F(" samples"));
            Serial.print(rotationCount);
            Serial.println(F(" collections"));
            if (missionTimeSecs >= ((rotationInterval * 60) * rotationCount) && missionTimeSecs < (((rotationInterval * 60) * rotationCount) + rotationDuration) && missionTimeSecs < (collectionDurationSecs)) // Test for Motor Rotation Time?
            {
                Serial.print(F("Motor is running Mission Time = "));
                analogWrite(motorPin, motorSpeed); // Turn ON Motor for rotationDuration
                motorCurrentMeasured = motorCurrentMeasured + readMotorI();
                motorOnFlag = 1;
                Serial.println(missionTimeSecs);
            }
            else if (motorOnFlag == 1) // if motor already running?
            {
                motorOnFlag = 0;
                analogWrite(motorPin, 0); // Turn Motor OFF
                motorCurrentMeasured = motorCurrentMeasured / rotationDuration; // average current reading
                if (motorCurrentMeasured > 300)
                {
                    badRotationCount++;
                }
                motorCurrentMeasuredLast = motorCurrentMeasured;
                motorCurrentMeasured = 0;
                rotationCount++;
            }
            if (missionTimeSecs >= (O2Interval *60 * O2SampleCount) && missionTimeSecs < missionDurationSecs)
            {
                Serial.print(F("O2 Sampling..."));
                writeDataToSD();
                //writeDataToSD();
                // writeDataToSD();
                Serial.print(F("O2 Sampling complete"));
                O2SampleCount++;
            }
            flashDuration = 500;
        }

        if (elapsedTimeSecs >= (startDelayHours * 3600) + missionDurationSecs)
        {
            Serial.println(F("Mission Complete"));
            while (Serial.availableForWrite())
            {
            }
            //noInterrupts();
            digitalWrite(ledPin, LOW);
            //delay(1000); // allow time for serial messages to complete
            detachInterrupt(1);
            sleep_enable();
            sleep_cpu();
        }

        Serial.println();
    }

}


//***************************************************
//function to clear serial buffer and wait for input
void serialClearAndWait()
{
    while (Serial.available() > 0) // empty buffer
    {
        Serial.read();
    }
    while (Serial.available() == 0) // wait for a character/s to be typed
    {
    }
}
//***************************************************************

//**************************************************************
//function to serially display time
void serialDateTimeDisplay()
{
    //Serial.println("Current Date and Time: ");
    DateTime now = RTC.now();
    if (now.day() >= 10)
    {
        Serial.print(now.day(), DEC);
        if (logFlag) logfile.print(now.day(), DEC);
    }
    else
    {
        Serial.print('0');
        Serial.print(now.day(), DEC);
        if (logFlag) logfile.print('0');
        if (logFlag) logfile.print(now.day(), DEC);
    }
    Serial.print("/");
    if (logFlag) logfile.print("/");


    if (now.month() >= 10)
    {
        Serial.print(now.month(), DEC);
        if (logFlag) logfile.print(now.month(), DEC);
    }
    else
    {
        Serial.print('0');
        Serial.print(now.month(), DEC);
        if (logFlag) logfile.print('0');
        if (logFlag) logfile.print(now.month(), DEC);
    }
    Serial.print("/");
    if (logFlag) logfile.print("/");
    Serial.print(now.year(), DEC);
    if (logFlag) logfile.print(now.year(), DEC);
    Serial.print(";");
    if (logFlag) logfile.print(";");
    if (now.hour() >= 10)
    {
        Serial.print(now.hour(), DEC);
        if (logFlag) logfile.print(now.hour(), DEC);
    }
    else
    {
        Serial.print('0');
        if (logFlag) logfile.print('0');
        Serial.print(now.hour(), DEC);
        if (logFlag) logfile.print(now.hour(), DEC);
    }
    Serial.print(":");
    if (logFlag) logfile.print(":");
    if (now.minute() >= 10)
    {
        Serial.print(now.minute(), DEC);
        if (logFlag) logfile.print(now.minute(), DEC);
    }
    else
    {
        Serial.print('0');
        if (logFlag) logfile.print('0');
        Serial.print(now.minute(), DEC);
        if (logFlag) logfile.print(now.minute(), DEC);
    }
    Serial.print(":");
    if (logFlag) logfile.print(":");
    if (now.second() >= 10)
    {
        Serial.println(now.second(), DEC);
        if (logFlag) logfile.print(now.second(), DEC);
    }
    else
    {
        Serial.print('0');
        if (logFlag) logfile.print('0');
        Serial.println(now.second(), DEC);
        if (logFlag) logfile.print(now.second(), DEC);

    }
    if (logFlag) logfile.print(";");
}
//*********************************************
void readBattV()
{
    Serial.print(F("Battery Voltage Now: "));
    int battVoltage = analogRead(battVoltPin);  //read battery voltage
    battVoltage = map(battVoltage, 0, 1023, 0, 1450);  //scale to read correctly
    Serial.print(battVoltage / 100); //as above
    if (logFlag) logfile.print(battVoltage / 100);
    Serial.print(".");
    if (logFlag) logfile.print(".");
    if (battVoltage % 100 < 10)
    {
        Serial.print("0");
        if (logFlag) logfile.print("0");
    }
    Serial.println(battVoltage % 100);
    if (logFlag) logfile.print(battVoltage % 100);
    if (logFlag) logfile.print(";");
}
//************************************************
void setRTC()
{
    serialClearAndWait();
    Serial.println(F("Enter Date DD/MM/YYYY then hit <return>"));
    delay(1000);
    serialClearAndWait();
    int rtcDay = Serial.parseInt();    // read the incoming byte and convert ascii to binary:
    int rtcMonth = Serial.parseInt();    // read the incoming byte and convert ascii to binary:
    int rtcYear = Serial.parseInt();    // read the incoming byte and convert ascii to binary:
    Serial.println(F("Enter Time HR:MIN then hit <return>"));
    delay(1000);
    serialClearAndWait();
    int rtcHour = Serial.parseInt();    // read the incoming byte and convert ascii to binary:
    int rtcMin = Serial.parseInt();    // read the incoming byte and convert ascii to binary:
    RTC.adjust(DateTime(rtcYear, rtcMonth, rtcDay, rtcHour, rtcMin, 0), MCP7941X_BATT_BKUP_EN);
    serialDateTimeDisplay();
    Serial.println(F(" DATE AND TIME SET!"));
}
//************************************************
void initOptode()
{
    mySerial.println("////////////");
    delay(1000);
    mySerial.println("Set Passkey(1)");
    delay(1000);
    mySerial.println("Set Enable Text(no)");
    delay(1000);
}
//************************************************
void initSDCard()
{
    Serial.print(F("Initializing SD card..."));
    if (!SD.begin(chipSelect))
    {
        Serial.println(F("Card failed, or not present"));
    }
    else
    {
        Serial.println("card initialized.");
    }
}
//*********************************************
//*********************************************

void openLogFile()
{
    char filename[] = "LOGGER00.CSV";
    for (int i = 0; i < 100; i++)
    {
        filename[6] = i / 10 + '0';
        filename[7] = i % 10 + '0';
        if (!SD.exists(filename))
        {
            // only open a new file if it doesn't exist
            logfile = SD.open(filename, FILE_WRITE);
            break;  // leave the loop!
        }
    }
    if (!logfile)
    {
        Serial.println(F("couldnt create file"));
    }

    Serial.print("Logging to: ");
    Serial.println(filename);

}
//************************************************
void writeDataToSD()
{
    if (!logfile) {
        Serial.println(F("couldnt open file"));
    }
    logfile.print(O2SampleCount);
    logfile.print(";");
    logfile.print(rotationCount - 1);
    logfile.print(";");
    logfile.print(badRotationCount);
    logfile.print(";");

    serialDateTimeDisplay();
    readBattV();

    while (mySerial.available() > 0)
    {
        mySerial.read();
    }


    mySerial.println("////////////");
    delay(1);
    mySerial.print("\r\n");
    mySerial.print("do sample\r\n");
    while (mySerial.available() == 0)
    {
        //wait for data
    }

    int params = 0;
    while ((mySerial.available() > 0))
    {
        params++;
        if (params >= 13) break;
        float val = mySerial.parseFloat();
        Serial.println(val);
        logfile.print(val, 3);
        logfile.print(";");
        delay(50);

    }

    logfile.println();
    logfile.flush();
}


//************************************************
void sendCmd(String cmd)
{
        while (mySerial.available() > 0)
        {
            mySerial.read();
        }
        mySerial.print(cmd+"\r\n");
        Serial.print(F("Command sent:"));
        Serial.println(cmd);
        Serial.print(F("Response:"));
        while (mySerial.available() == 0)
        {
            //wait for data
        }
        while ((mySerial.available() > 0))
        {
            Serial.println(mySerial.readString());
        }
}

//************************************************
int readDataFromSD()
{
    serialClearAndWait();
    char fileName[] = "LOGGER00.CSV";
    if (Serial.available())
    {
        fileName[6] = Serial.read();
        if (fileName[6] < '0' || fileName[6] > '9')
        {
            Serial.println("Syntax error!");
            return 1;
        }
    }
    serialClearAndWait();
    if (Serial.available())
    {
        fileName[7] = Serial.read();
        if (fileName[7] < '0' || fileName[7] > '9')
        {
            Serial.println("Syntax error!");
            return 1;
        }
    }
    Serial.println(fileName);
    if (!SD.exists(fileName))
    {
        Serial.println(F("Error, Filename doesn't exist!"));
        return 1;
    }
    logfile = SD.open(fileName);
    if (logfile)
    {
        Serial.print("Opening ");
        Serial.print(fileName);
        Serial.println("...");
        while (logfile.available())
        {
            Serial.write(logfile.read());
        }
        logfile.close();
    }
    else
    {
        Serial.println("error reading file");
    }
    return 0;
}
//************************************************
void printDirectory(File dir, int numTabs) {
    while (true) {

        File entry = dir.openNextFile();
        if (!entry) {
            // no more files
            break;
        }
        for (uint8_t i = 0; i < numTabs; i++) {
            Serial.print('\t');
        }
        Serial.print(entry.name());
        if (entry.isDirectory()) {
            Serial.println("/");
            printDirectory(entry, numTabs + 1);
        }
        else {
            // files have sizes, directories do not
            Serial.print("\t\t");
            Serial.println(entry.size(), DEC);
        }
        entry.close();
    }
}
//************************************************
void rotateBallOnce()
{
    rotationDuration = EEPROM.read(13);
    Serial.print("please wait...Rotating Ball For Rotation Duration (");
    Serial.print(rotationDuration);
    Serial.println(" secs)");
    analogWrite(motorPin, motorSpeed); // Turn ON Motor for rotationDuration
    for (int i = 0; i < rotationDuration; i++)
    {
        Serial.println(i);
        delay(1000);
    }
    analogWrite(motorPin, 0); // Turn Motor OFF
}
//************************************************
int readMotorI()
{
    int motVoltage = analogRead(motorCurrent);  //read motor current(volts across 100R)
    return motVoltage;
}
//************************************************
