// YL-39 + YL-69 humidity sensor
byte humidity_sensor_pin = A1;
byte humidity_sensor_vcc = 19;
byte humidity_sensor_gnd = 17;
byte motor_pin = 3;

void setup() {
  // Init the humidity sensor board
  pinMode(humidity_sensor_vcc, OUTPUT);
  pinMode(humidity_sensor_gnd, OUTPUT);
  pinMode(motor_pin, OUTPUT);
  digitalWrite(humidity_sensor_vcc, HIGH);
  digitalWrite(humidity_sensor_gnd, LOW);
  digitalWrite(motor_pin, LOW);

  // Setup Serial
  while (!Serial);
  delay(1000);
  Serial.begin(115200);
}

int read_humidity_sensor() {
  digitalWrite(humidity_sensor_vcc, HIGH);
  delay(500);
  int value = analogRead(humidity_sensor_pin);
  digitalWrite(humidity_sensor_vcc, LOW);
  return 1023 - value;
}

void loop() {
  Serial.print("Humidity Level (0-1023): ");
  Serial.println(read_humidity_sensor());
  delay(1000);
  digitalWrite(motor_pin, HIGH);
  digitalWrite(13, HIGH);
  delay(5000);
  digitalWrite(motor_pin, LOW);
  digitalWrite(13, LOW);
  delay(1000);
}
