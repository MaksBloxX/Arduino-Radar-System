#include <Servo.h>

Servo radarServo;

// ================= PIN SETTINGS =================

const byte SERVO_PIN = 9;

const byte TRIG_PIN = 10;
const byte ECHO_PIN = 11;

const byte GREEN_LED = 6;
const byte RED_LED = 7;

// ================= RADAR SETTINGS =================

int angle = 0;
int direction = 1;

int servoDelay = 30;

bool radarRunning = false;

// Detection distance
const int DETECTION_DISTANCE = 50;


// ================= GET DISTANCE =================

int getDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    30000
  );

  if (duration == 0) {
    return 400;
  }

  int distance =
    duration * 0.0343 / 2;

  return distance;
}


// ================= COMMAND HANDLER =================

void checkCommands() {

  if (Serial.available()) {

    String command =
      Serial.readStringUntil('\n');

    command.trim();

    // START RADAR
    if (command == "START") {

      radarRunning = true;
    }

    // STOP RADAR
    else if (command == "STOP") {

      radarRunning = false;

      radarServo.write(90);

      // Both LEDs OFF
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, LOW);
    }

    // SPEED
    else if (command.startsWith("SPEED:")) {

      int newSpeed =
        command.substring(6).toInt();

      if (newSpeed >= 10 &&
          newSpeed <= 200) {

        servoDelay = newSpeed;
      }
    }
  }
}


// ================= SETUP =================

void setup() {

  Serial.begin(9600);

  radarServo.attach(
    SERVO_PIN
  );

  pinMode(
    TRIG_PIN,
    OUTPUT
  );

  pinMode(
    ECHO_PIN,
    INPUT
  );

  pinMode(
    GREEN_LED,
    OUTPUT
  );

  pinMode(
    RED_LED,
    OUTPUT
  );

  // Initial state
  digitalWrite(
    GREEN_LED,
    LOW
  );

  digitalWrite(
    RED_LED,
    LOW
  );

  radarServo.write(90);

  delay(500);
}


// ================= LOOP =================

void loop() {

  // Check commands from Python
  checkCommands();

  // ================= STOPPED =================

  if (!radarRunning) {

    digitalWrite(
      GREEN_LED,
      LOW
    );

    digitalWrite(
      RED_LED,
      LOW
    );

    delay(5);

    return;
  }


  // ================= RUNNING =================

  radarServo.write(angle);

  delay(servoDelay);


  // Measure distance
  int distance =
    getDistance();


  // ================= LED STATUS =================

  if (distance <= DETECTION_DISTANCE &&
      distance > 0) {

    // Object detected
    digitalWrite(
      GREEN_LED,
      LOW
    );

    digitalWrite(
      RED_LED,
      HIGH
    );

  } else {

    // Clear
    digitalWrite(
      GREEN_LED,
      HIGH
    );

    digitalWrite(
      RED_LED,
      LOW
    );

  }


  // ================= SEND DATA =================

  Serial.print(angle);

  Serial.print(",");

  Serial.println(distance);


  // ================= CHANGE DIRECTION =================

  angle += direction;


  if (angle >= 180) {

    angle = 180;

    direction = -1;
  }


  if (angle <= 0) {

    angle = 0;

    direction = 1;
  }
}
