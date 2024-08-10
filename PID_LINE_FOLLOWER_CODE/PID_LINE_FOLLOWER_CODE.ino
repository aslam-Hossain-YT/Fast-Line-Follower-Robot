//Motor pins
#define rmf 6
#define rmb 5
#define lmf 4
#define lmb 2
//EnA and EnB
#define rms 9  //EnA
#define lms 3  //EnB

int sensor[6];  //to store the sensor value
int threshold = 300;
float c;
int left_motor_speed = 200, right_motor_speed = 200;
int left_motor, right_motor;
int kp = 40, kd = 1000;
int PID_value;
float current_error, previous_error;
int turn_speed = 120;
char t;

//push button and led
int button1 = 8;
int button2 = 9;
int button3 = 10;
int button4 = 11;
int led12 = 12;
int led = 13;
bool button1_state, button2_state, button3_state, button4_state;


void setup() {
  //motor driver pins as output
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);

  //button pin as input
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  //led pin as output
  pinMode(led12, OUTPUT);
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}


void loop() {
  digitalWrite(led, HIGH);
  button_status();  //sensor_reading the status of push button

  //when button 1 will be pressed then the robot will start to follow lines.
  while (button1_state != 1) {
    digitalWrite(led, LOW);

    //blink led12
    for (int i = 0; i < 5; ++i) {
      digitalWrite(led12, HIGH);
      delay(100);
      digitalWrite(led12, LOW);
      delay(100);
    }

    Line_Follow();  //line follow using pid
  }

  //when push button 2 is pressed.
  //to check if the both motor are running in the same speed or not.
  //if the both runs in same speed then the robot will go straight otherwise it will move in left or right.
  while (button2_state != 1) {
    digitalWrite(led, LOW);

    for (int i = 0; i < 3; ++i) {
      digitalWrite(led12, HIGH);
      delay(500);
      digitalWrite(led12, LOW);
      delay(500);
    }
    motor(250, 250);
    delay(3000);
    motor(0, 0);
    button_status();
  }

  while (button4_state != 1) {
    digitalWrite(led, LOW);
    (button3_state = digitalRead(button3) != 1) ? button3_state = 0, button4_state = 1 : button4_state = 0;

    show_analog_value();
  }

  //stop mode if button 3 is pressed
  while (button3_state != 1) {
    motor(0, 0);
    digitalWrite(led, HIGH);
    button_status();
  }
}

void sensor_reading() {
  float a;
  float b;
  for (int i = 0; i < 6; i++) {
    if (i < 4) {  //analog read from arduino analog pins except A4 and A5
      sensor[i] = analogRead(i);
    } else {
      sensor[i] = analogRead(i + 2);
    }
    if (sensor[i] > threshold) {
      sensor[i] = 1;
    } else {
      sensor[i] = 0;
    }
  }
  a = (sensor[0] * 1 + sensor[1] * 2 + sensor[2] * 3 + sensor[3] * 4 + sensor[4] * 5 + sensor[5] * 6);
  b = (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5]);
  if (b > 0) c = a / b;
}

void Line_Follow() {
  while (1) {
    sensor_reading();
    //Straight Line Follow
    if (sensor[2] == 1 || sensor[3] == 1) {
      current_error = 3.5 - c;
      PID_value = current_error * kp + kd * (current_error - previous_error);
      previous_error = current_error;

      right_motor = right_motor_speed - PID_value;
      left_motor = left_motor_speed + PID_value;
      motor(left_motor, right_motor);
    }

    //all sensor in white surface
    if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 0) {
      if (t == 'r') right();
      else if (t == 'l') left();
      else U_turn();
    }

    //Right Turn
    if (sensor[5] == 0 && sensor[0] == 1) t = 'r';
    //Left Turn
    if (sensor[5] == 1 && sensor[0] == 0) t = 'l';

    //all sensor in black surface
    if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1) {
      delay(30);
      sensor_reading();
      if ((sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5]) == 6) {
        motor(0, 0);  //stop
        while ((sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5]) == 6) sensor_reading();
      } else if ((sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5]) == 0) t = 'r';
    }
  }
}

void right() {
  while (1) {
    motor(turn_speed, -turn_speed);
    while (sensor[2] == 0 && sensor[3] == 0) sensor_reading();
    motor(-turn_speed, turn_speed);
    delay(20);
    break;
  }
}

void left() {
  while (1) {
    motor(-turn_speed, turn_speed);
    while (sensor[2] == 0 && sensor[3] == 0) sensor_reading();
    motor(turn_speed, -turn_speed);
    delay(20);
    break;
  }
}
void U_turn() {
  while (1) {
    delay(120);
    digitalWrite(led, HIGH);
    motor(-turn_speed, turn_speed);
    while (sensor[2] == 0 && sensor[3] == 0) sensor_reading();
    motor(turn_speed, -turn_speed);
    delay(20);
    digitalWrite(led, LOW);
    break;
  }
}

//sensor_reading the status of push button and saving the state.
void button_status() {
  button1_state = digitalRead(button1);
  button2_state = digitalRead(button2);
  button3_state = digitalRead(button3);
  button4_state = digitalRead(button4);
}

//motor run function
void motor(int a, int b) {
  if (a > 0) {
    digitalWrite(lmf, 1);
    digitalWrite(lmb, 0);
  } else {
    a = -(a);
    digitalWrite(lmf, 0);
    digitalWrite(lmb, 1);
  }
  if (b > 0) {
    digitalWrite(rmf, 1);
    digitalWrite(rmb, 0);
  } else {
    b = -(b);
    digitalWrite(rmf, 0);
    digitalWrite(rmb, 1);
  }

  if (a > 250) a = 250;
  if (b > 250) b = 250;

  analogWrite(lms, a);
  analogWrite(rms, b);
}

void show_analog_value() {
  for (short int i = 5; i >= 0; i--) {
    if (i > 3) Serial.print(String(analogRead(i + 2)) + " ");
    else Serial.print(String(analogRead(i)) + " ");
  }
  delay(100);
  Serial.println();
}
