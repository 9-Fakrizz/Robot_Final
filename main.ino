#include <ESP32Servo.h>
#include <math.h>

#define PUL_PIN1 12  // Step (pulse) pin for Stepper 1
#define DIR_PIN1 14  // Direction pin for Stepper 1
#define PUL_PIN2 15  // Step (pulse) pin for Stepper 2
#define DIR_PIN2 2   // Direction pin for Stepper 2

#define IN1_PIN 18  // Motor 1 control
#define IN2_PIN 19  // Motor 1 control
#define IN3_PIN 17  // Motor 2 control
#define IN4_PIN 5   // Motor 2 control

#define gripper_pin 16
#define wrist_pin 4

#define limSwitch_1 27
#define limSwitch_2 26

int stepDelay = 1200;  // Delay in microseconds (controls speed)

Servo gripper;
Servo wrist;

// float x = 0;  // Test X position
// float y = 220.0;  // Test Y position
// float z = 220.0;   // Test Z position

struct answer {
  float theta0 = 0;
  float theta1 = 0;
  float theta2 = 0;
  float theta3 = 0;
};

struct link_robot_t {
  float l1 = 270.0;
  float l2 = 180.0;
  float l3 = 100.0;
};

link_robot_t robot_link_1 = {};

float theta1, theta2, theta3;
float x_input = 0;
float y_input = 0;
float z_input = 0;
float ee_angle = 0;

int current_angle_link1 = 0;
int current_angle_link2 = 0;
int current_angle_link3 = 0;
bool gripped = false;
bool cam_position = false;

void setHome(){
  // Set home
  digitalWrite(DIR_PIN1, LOW);
  for (int i = 0; i < 1500; i++) {
    digitalWrite(PUL_PIN1, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(PUL_PIN1, LOW);
    delayMicroseconds(stepDelay);
  }
  Serial.println("Set Home link 1");
  handleServoCommand("s1,180");
  Serial.println("Set Home wrist");

  current_angle_link1 = 112;
  current_angle_link2 = -90;
  current_angle_link3 = -112;
}

void SetZeroStepper(){
  digitalWrite(DIR_PIN1, HIGH);
  digitalWrite(DIR_PIN2, HIGH);
  
  //Set zero link 2
  while(digitalRead(limSwitch_2)!=0){
    digitalWrite(PUL_PIN2, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(PUL_PIN2, LOW);
    delayMicroseconds(stepDelay);
    Serial.println("lim2 :" + String(digitalRead(limSwitch_1)));
  }
  Serial.println("Complete Set Zero 2");
  
  //Set zero link 1
  delay(500);
  while(digitalRead(limSwitch_1)!=0){
    digitalWrite(PUL_PIN1, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(PUL_PIN1, LOW);
    delayMicroseconds(stepDelay);
    Serial.println("lim1 :" + String(digitalRead(limSwitch_1)));
  }
  Serial.println("Complete Set Zero 1");
  delay(1000);
  setHome();
}

// Function to handle serial input
void handleSerial() {
  char receivedData[30];
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read input until newline
    input.trim();  // Remove whitespace

    if(cam_position == false){
      input = "0,220,220,-50";
      cam_position = true;
    }

    if (input.startsWith("1,") || input.startsWith("2,")) {
        handleStepperCommand(input);
    }else if (input.startsWith("s1,") || input.startsWith("s2,")) {
        handleServoCommand(input);
    }else if (input == "forward") {
        moveForward();
    } else if (input == "backward") {
        moveBackward();
    } else if (input == "CW") {
        rotateLeft();
    } else if (input == "CCW") {
        rotateRight();
    } else if (input == "stop") {
        stopMotors();
    }else if (input == "SCW") {
        moveBackward();
        delay(400);
        rotateLeft();
        delay(2000);
        stopMotors();
    }else if (input == "SCCW") {
        moveBackward();
        delay(400);
        rotateRight();
        delay(2000);
        stopMotors();
    } 
    else if(sscanf(input.c_str(), "%f,%f,%f,%f", &x_input, &y_input, &z_input, &ee_angle) == 4) {

      Serial.printf("Input is x=%d, y=%d, z=%d, ee_ang=%d\n", int(x_input), int(y_input), int(z_input), int(ee_angle));
      answer output_answer_1 = calculate_IK(x_input, y_input, z_input, ee_angle, robot_link_1, 1);
      answer output_answer_2 = calculate_IK(x_input, y_input, z_input, ee_angle, robot_link_1, -1);
      
      // Print the results
      Serial.println("Inverse Kinematics Results (Solution 1):");
      Serial.print("Theta 0: "); Serial.println(output_answer_1.theta0);
      Serial.print("Theta 1: "); Serial.println(output_answer_1.theta1);
      Serial.print("Theta 2: "); Serial.println(output_answer_1.theta2);
      Serial.print("Theta 3: "); Serial.println(output_answer_1.theta3);
      
      Serial.println("\nInverse Kinematics Results (Solution 2):");
      Serial.print("Theta 0: "); Serial.println(output_answer_2.theta0);
      Serial.print("Theta 1: "); Serial.println(output_answer_2.theta1);
      Serial.print("Theta 2: "); Serial.println(output_answer_2.theta2);
      Serial.print("Theta 3: "); Serial.println(output_answer_2.theta3);

      int final_theta1 = 0;
      int final_theta2 = 0;
      int final_theta3 = 0;
      bool sol2_active = false;
      bool activate = false;
      if(output_answer_1.theta1 < 0 || output_answer_1.theta1 > 180 ){
        Serial.println("(sol 1) deadzone theta 1 !");
        sol2_active = true;
      }else if(output_answer_1.theta2 < -90 || output_answer_1.theta2 > 90){
        Serial.println("(sol 1) deadzone theta 2 !");
        sol2_active = true;
      }else if(output_answer_1.theta3 < -135 || output_answer_1.theta3 > 45){
        Serial.println("(sol 1) deadzone theta 3 !");
        sol2_active = true;
      }else{
        final_theta1 = int(output_answer_1.theta1);
        final_theta2 = int(output_answer_1.theta2);
        final_theta3 = int(output_answer_1.theta3);
        activate = true;
      }

      if(sol2_active){
        if(output_answer_2.theta1 < 0 || output_answer_2.theta1 > 180 ){
          Serial.println("(sol 2) deadzone theta 1 !");
          sol2_active = true;
        }else if(output_answer_2.theta2 < -90 || output_answer_2.theta2 > 90){
          Serial.println("(sol 2) deadzone theta 2 !");
          sol2_active = true;
        }else if(output_answer_2.theta3 < -135 || output_answer_2.theta3 > 45){
          Serial.println("(sol 2) deadzone theta 3 !");
          sol2_active = true;
        }else{
          final_theta1 = int(output_answer_2.theta1);
          final_theta2 = int(output_answer_2.theta2);
          final_theta3 = int(output_answer_2.theta3);
          activate = true;
        }
      }
      if (activate) {
        Serial.printf("Final is Shoulder=%d°, Elbow=%d°, Wrist=%d°\n", final_theta1, final_theta2, final_theta3);

        int move_l1 = (current_angle_link1 - final_theta1);
        int move_l2 = (current_angle_link2 - final_theta2);
        int move_l3 = 50 - final_theta3;

        Serial.printf("move1=%d°, move2=%d°, move3=%d°\n", move_l1, move_l2, move_l3);

        // Store values in an array of structs
        struct Move {
            int value;
            String command;
        };

        Move moves[] = {
            {abs(move_l1), String("1,") + (move_l1 >= 0 ? "+" : "") + String(move_l1 * 13.333)},
            {abs(move_l2), String("2,") + (move_l2 >= 0 ? "+" : "") + String(move_l2 * 13.333)},
            {abs(move_l3), "s1," + String(move_l3)}
        };

        // Sort the moves by absolute value
        for (int i = 0; i < 2; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (moves[i].value > moves[j].value) {
                    Move temp = moves[i];
                    moves[i] = moves[j];
                    moves[j] = temp;
                }
            }
        }

        // Execute movements in order
        for (int i = 0; i < 3; i++) {
            if (moves[i].command.startsWith("s1")) {
                handleServoCommand(moves[i].command);
                current_angle_link3 = final_theta3;
            } else {
                handleStepperCommand(moves[i].command);
                if (moves[i].command.startsWith("1")) {
                    current_angle_link1 = final_theta1;
                } else {
                    current_angle_link2 = final_theta2;
                }
            }
        }
      }
    }
    else {
        Serial.println("Invalid command.");
    }
  }
}

// Function to handle stepper motor commands
void handleStepperCommand(String input) {
    int commaIndex = input.indexOf(',');  // Find comma position
    if (commaIndex == -1) {
        Serial.println("Invalid input. Use format: motor,direction+steps");
        return;
    }

    int motorNumber = input.substring(0, commaIndex).toInt();  // Get motor number
    char direction = input.charAt(commaIndex + 1);  // Get direction (+ or -)
    int numSteps = input.substring(commaIndex + 2).toInt();  // Get step count

    if (numSteps > 0) {
        if (motorNumber == 1) {
            digitalWrite(DIR_PIN1, (direction == '+') ? HIGH : LOW);
            moveStepper(PUL_PIN1, numSteps);
        } else if (motorNumber == 2) {
            digitalWrite(DIR_PIN2, (direction == '+') ? HIGH : LOW);
            moveStepper(PUL_PIN2, numSteps);
        } else {
            Serial.println("Invalid motor number. Use 1 or 2.");
        }
    } else {
        Serial.println("Invalid step count.");
    }
}

// Function to move a specific stepper motor
void moveStepper(int pulPin, int numSteps) {
    for (int i = 0; i < numSteps; i++) {
        digitalWrite(pulPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(pulPin, LOW);
        delayMicroseconds(stepDelay);
    }
}

// Functions for controlling normal motors
void moveForward() {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    Serial.println("Moving forward");
}

void moveBackward() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    Serial.println("Moving backward");
}

void rotateLeft() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    Serial.println("Rotating left");
}

void rotateRight() {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    Serial.println("Rotating right");
}

void stopMotors() {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    Serial.println("Motors stopped");
}

// Function to handle servo commands with smooth movement
void handleServoCommand(String input) {
    int commaIndex = input.indexOf(',');  // Find comma position
    if (commaIndex == -1) {
        Serial.println("Invalid input. Use format: s1,angle or s2,angle");
        return;
    }

    int servoNumber = input.substring(1, commaIndex).toInt();  // Get servo number
    int targetAngle = input.substring(commaIndex + 1).toInt();  // Get target angle
    int delayTime = 10;  // Delay in milliseconds to slow down movement

    if (targetAngle >= 0 && targetAngle <= 360) {
        if (servoNumber == 1) {
            int currentAngle = wrist.read();  // Get current position
            if (currentAngle < targetAngle) {
                for (int angle = currentAngle; angle <= targetAngle; angle++) {
                    wrist.write(angle);
                    delay(delayTime);
                }
            } else {
                for (int angle = currentAngle; angle >= targetAngle; angle--) {
                    wrist.write(angle);
                    delay(delayTime);
                }
            }
            Serial.print("Wrist moved to ");
            Serial.println(targetAngle);
        } 
        else if (servoNumber == 2) {
            int currentAngle = gripper.read();  // Get current position
            if (currentAngle < targetAngle) {
                for (int angle = currentAngle; angle <= targetAngle; angle++) {
                    gripper.write(angle);
                    delay(delayTime);
                }
            } else {
                for (int angle = currentAngle; angle >= targetAngle; angle--) {
                    gripper.write(angle);
                    delay(delayTime);
                }
            }
            Serial.print("Gripper moved to ");
            Serial.println(targetAngle);
        } 
        else {
            Serial.println("Invalid servo number. Use s1 or s2.");
        }
    } else {
        Serial.println("Invalid angle. Use a value between 0 and 180.");
    }
}

answer calculate_IK(float x, float y, float z, float end_effector_angle, link_robot_t robot_link_1, int solution) {
  float l1 = robot_link_1.l1;
  float l2 = robot_link_1.l2;
  float l3 = robot_link_1.l3; // Now representing end-effector length

  float w = sqrt(pow(x, 2) + pow(y, 2)) - l3 * cos(end_effector_angle * M_PI / 180.0);
  float d = z - l3 * sin(end_effector_angle * M_PI / 180.0);

  float a = pow(w, 2) + pow(d, 2) - pow(l1, 2) - pow(l2, 2);
  float b = 2 * l1 * l2;
  float c = a / b;

  // Prevent invalid acos input
  c = constrain(c, -1.0, 1.0);
  float theta2_1 = acos(c); // Elbow-Up Solution
  float theta2_2 = -theta2_1; // Elbow-Down Solution

  float a1_1 = l1 + l2 * cos(theta2_1);
  float b1_1 = l2 * sin(theta2_1);
  float theta1_1 = atan2(d * a1_1 - w * b1_1, w * a1_1 + d * b1_1);

  float a1_2 = l1 + l2 * cos(theta2_2);
  float b1_2 = l2 * sin(theta2_2);
  float theta1_2 = atan2(d * a1_2 - w * b1_2, w * a1_2 + d * b1_2);

  float theta0 = atan2(x, y);

  // Convert to degrees before storing values
  theta0 = theta0 * 180.0 / M_PI;
  theta1_1 = theta1_1 * 180.0 / M_PI;
  theta2_1 = theta2_1 * 180.0 / M_PI;
  theta1_2 = theta1_2 * 180.0 / M_PI;
  theta2_2 = theta2_2 * 180.0 / M_PI;

  // Use end-effector angle directly
  // float theta3 = end_effector_angle;

  answer output_answer;
  if (solution == 1) {
    output_answer.theta0 = theta0;
    output_answer.theta1 = theta1_1;
    output_answer.theta2 = theta2_1;
    float theta3 = end_effector_angle - (theta1_1 + theta2_1);
    output_answer.theta3 = theta3;
  } else {
    output_answer.theta0 = theta0;
    output_answer.theta1 = theta1_2;
    output_answer.theta2 = theta2_2;
    float theta3 = end_effector_angle - (theta1_2 + theta2_2);
    output_answer.theta3 = theta3;
  }
  return output_answer;
}
void setup() {
    pinMode(PUL_PIN1, OUTPUT);
    pinMode(DIR_PIN1, OUTPUT);
    pinMode(PUL_PIN2, OUTPUT);
    pinMode(DIR_PIN2, OUTPUT);

    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    pinMode(limSwitch_1, INPUT_PULLUP);
    pinMode(limSwitch_2, INPUT_PULLUP);

    gripper.attach(gripper_pin);
    wrist.attach(wrist_pin);

    Serial.begin(115200);  // Start serial communication

    gripper.write(0);
    wrist.write(0);
    SetZeroStepper();

    // gripper.write(90);
    wrist.write(180);
    delay(500);
    gripper.write(180);
    // delay(500);
    // gripper.write(270);
    // delay(500);
    // gripper.write(360);
    // delay(500);
    // gripper.write(0);
    String cmd = "0,220,220,-70";

}

void loop() {
    handleSerial();
    delay(100);
}