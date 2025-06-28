#define DT_PIN 2
#define SCK_PIN 3

const int EN1 = 9;
const int IN1 = 7;
const int IN2 = 8;

double Kp = 0.5;
double Ki = 0;
double Kd = 0;

double setpoint = 1000.0;
double input = 0;
double output = 0;
double error = 0;
double lastError = 0;
double integral = 0;
unsigned long lastTime = 0;


void setup() {
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  scale.begin(DT_PIN, SCK_PIN);
  scale.set_gain(32);

  Serial.begin(9600);
  Serial.println("Ready for PID and setpoint commands.");
}

void loop() {

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("PID,")) {
      int first = cmd.indexOf(',') + 1;
      int second = cmd.indexOf(',', first);
      int third = cmd.indexOf(',', second + 1);

      if (first > 0 && second > first && third > second) {
        Kp = cmd.substring(first, second).toFloat();
        Ki = cmd.substring(second + 1, third).toFloat();
        Kd = cmd.substring(third + 1).toFloat();

        Serial.print("PID updated: ");
        Serial.print(Kp); Serial.print(", ");
        Serial.print(Ki); Serial.print(", ");
        Serial.println(Kd);
      }
    } else if (cmd.startsWith("SET,")) {
      int idx = cmd.indexOf(',') + 1;
      if (idx > 0) {
        setpoint = cmd.substring(idx).toFloat();
        Serial.print("Setpoint updated: ");
        Serial.println(setpoint);
      }
    }
  }

  int sensor = analogRead(A7);
  input = 0.006 * pow(sensor, 2) + 24.521 * sensor - 2319.569;

  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt == 0) dt = 0.001;

  error = setpoint - input;

  integral += error * dt;
  double derivative = (error - lastError) / dt;

  output = Kp * error + Ki * integral + Kd * derivative;
  int final_out = output + input;

  if (output > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, abs(output));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, abs(output));
  }

  lastError = error;
  lastTime = now;

  Serial.println(input);
  
}
