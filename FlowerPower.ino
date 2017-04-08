#include <PID_v1.h>

// YL-39 + YL-69 humidity sensor
static const int humidity_sensor_pin = A1;
static const int humidity_sensor_vcc = 19;
static const int humidity_sensor_gnd = 17;
static const int btn_pin = 12;
static const int btn_gnd = 10;
static const int relayPin1 = 3;
static const int test_led = 13;
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

void setup() {
    pinMode(humidity_sensor_pin, INPUT);
    pinMode(humidity_sensor_vcc, OUTPUT);
    pinMode(humidity_sensor_gnd, OUTPUT);
    pinMode(test_led, OUTPUT);
    pinMode(relayPin1, OUTPUT);
    pinMode(btn_pin, INPUT_PULLUP);
    pinMode(btn_gnd, OUTPUT);

    digitalWrite(humidity_sensor_vcc, LOW);
    digitalWrite(humidity_sensor_gnd, LOW);
    digitalWrite(relayPin1, LOW);
    digitalWrite(btn_gnd, LOW);
    digitalWrite(test_led, LOW);

    //input1 = analogRead(0);
    myPID1.SetMode(AUTOMATIC);
    myPID1.SetOutputLimits(0, windowSize);

    // connect AREF to 3.3V and use that as VCC, less noisy!
    //analogReference(EXTERNAL);

    //initialize the variables we're linked to
    setpoint1 = 125;

    // Setup Serial
    while (!Serial);
    delay(1000);
    Serial.begin(115200);
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

