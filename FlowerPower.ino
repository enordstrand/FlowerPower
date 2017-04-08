#include <PID_v1.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h> // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads

// YL-39 + YL-69 humidity sensor
static const int humidity_sensor_pin = A1;
static const int humidity_sensor_vcc = 19;
static const int humidity_sensor_gnd = 17;
static const int btn_pin = 12;
static const int btn_gnd = 10;
static const int relayPin1 = 3;
static const int test_led = 13;
static const int lcd_vcc = 8;
static const int lcd_gnd = 6;
static const long windowSize = 60000;

double setpoint1, input1, output1;
double kp=2000, ki=0, kd=0;
long now, windowStartTime;
long counter = 0;
double sensorValue1 = 0;
double meansensorValue1 = 0;
double humidity1 = 0;
const int numberOfSamples = 200;

int samples1[numberOfSamples];

PID myPID1(&humidity1, &output1, &setpoint1, kp, ki, kd, DIRECT);

// set the LCD address to 0x27 for a 16 chars 2 line display
// A FEW use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

void setup() {
    // Setup Serial
    while (!Serial);
    delay(1000);
    Serial.begin(115200);
  
    pinMode(humidity_sensor_pin, INPUT);
    pinMode(humidity_sensor_vcc, OUTPUT);
    pinMode(humidity_sensor_gnd, OUTPUT);
    pinMode(test_led, OUTPUT);
    pinMode(relayPin1, OUTPUT);
    pinMode(btn_pin, INPUT_PULLUP);
    pinMode(btn_gnd, OUTPUT);
    pinMode(lcd_vcc, OUTPUT);
    pinMode(lcd_gnd, OUTPUT);

    digitalWrite(humidity_sensor_vcc, LOW);
    digitalWrite(humidity_sensor_gnd, LOW);
    digitalWrite(relayPin1, LOW);
    digitalWrite(btn_gnd, LOW);
    digitalWrite(test_led, LOW);
    digitalWrite(lcd_vcc, HIGH);
    digitalWrite(lcd_gnd, LOW);

    lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight

    //input1 = analogRead(0);
    myPID1.SetMode(AUTOMATIC);
    myPID1.SetOutputLimits(0, windowSize);

    // ------- Quick 3 blinks of backlight  -------------
    for(int i = 0; i< 3; i++)
    {
        lcd.backlight();
        delay(250);
        lcd.noBacklight();
        delay(250);
    }
    lcd.backlight(); // finish with backlight on  

//-------- Write characters on the display ------------------
// NOTE: Cursor Position: (CHAR, LINE) start at 0  
    lcd.setCursor(0,0); //Start at character 4 on line 0
    lcd.print("Hello, world!");
    delay(1000);
    lcd.setCursor(0,1);
    lcd.print("HI!YourDuino.com");
    delay(8000);  

// Wait and then tell user they can start the Serial Monitor and type in characters to
// Display. (Set Serial Monitor option to "No Line Ending")
    lcd.clear();
    lcd.setCursor(0,0); //Start at character 0 on line 0
    lcd.print("Use Serial Mon");
    lcd.setCursor(0,1);
    lcd.print("Type to display");  
    

    // connect AREF to 3.3V and use that as VCC, less noisy!
    //analogReference(EXTERNAL);

    //initialize the variables we're linked to
    setpoint1 = 125;

    
}

void loop() {
    uint8_t i;
    float averageAnalogRead_1;

    //read the pushbutton value into a variable
    int sensorVal = digitalRead(btn_pin);
    //print out the value of the pushbutton
    Serial.println(sensorVal);

    if (sensorVal == HIGH) {
        digitalWrite(test_led, LOW);
    } else {
        digitalWrite(test_led, HIGH);
    }
   
    // take N samples in a row, with a slight delay
    //digitalWrite(humidity_sensor_vcc, HIGH);
    //delay(500);
    for (i=0; i< numberOfSamples; i++) {
       samples1[i] = read_humidity_sensor();
//     delay(10);
    }
    //digitalWrite(humidity_sensor_vcc, LOW);
   
    // average all the samples out
    averageAnalogRead_1 = 0;
    for (i=0; i< numberOfSamples; i++) {
       averageAnalogRead_1 += samples1[i];
    }
    averageAnalogRead_1 /= numberOfSamples;

    humidity1 = averageAnalogRead_1;
    
    doPid();

    // when characters arrive over the serial port...
    if (Serial.available()) {
      // wait a bit for the entire message to arrive
      delay(100);
      // clear the screen
      lcd.clear();
      // read all the available characters
      while (Serial.available() > 0) {
        // display each character to the LCD
        lcd.write(Serial.read());
      }
    }
}

void doPid() {
    myPID1.Compute();
    
    now = millis();
    if (now - windowStartTime > windowSize) {
        windowStartTime += windowSize;
    }

    if (output1 > 3000) {
      output1 = 3000;
    }
  
    if ((output1 > now - windowStartTime) && output1 > 100 && humidity1 > -1) {
        digitalWrite(relayPin1, HIGH);
        digitalWrite(13, HIGH);
        Serial.print("H1 ");
    } else {
        digitalWrite(relayPin1, LOW);
        digitalWrite(13, LOW);
        Serial.print("L1 ");
    }

    Serial.print("Hum1: ");
    Serial.print(humidity1);
    Serial.print(" W1: ");
    Serial.print(output1);
    Serial.print(" T1: ");
    Serial.print(now);
    Serial.print(" WT: ");
    Serial.println(now - windowStartTime);
}

int read_humidity_sensor() {
  int value = analogRead(humidity_sensor_pin);
  //return 1023 - value;
  return 123;
}

