#define rmf 4  //IN1
#define rmb 7  //IN2
#define lmf 3  //IN3
#define lmb 2  //IN4
#define rms 6  //EnA
#define lms 5  //EnB
  
//declaration of necessary variable to line follow
int s[6];
int threshold = 512, sensor_position;
float current_error, prev_current_error;
float average;
char turn;

//push button and led
int button1 = 8;
int button2 = 9;
int button3 = 10;
int button4 = 11;
int led12 = 12;
int led13 = 13;
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
  pinMode(led13, OUTPUT);
  Serial.begin(9600);
}


void loop() {
  digitalWrite(led13, HIGH);
  button_status();  //sensor_reading the status of push button

  //when button 1 will be pressed then the robot will start to follow lines.
  while (button1_state != 1) {
    digitalWrite(led13, LOW);

    //blink led12
    for (int i = 0; i < 5; ++i) {
      digitalWrite(led12, HIGH);
      delay(100);
      digitalWrite(led12, LOW);
      delay(100);
    }

    PID_LINE_FOLLOW();  //line follow using pid
  }

  //when push button 2 is pressed.
  //to check if the both motor are running in the same speed or not.
  //if the both runs in same speed then the robot will go straight otherwise it will move in left or right.
  while (button2_state != 1) {
    digitalWrite(led13, LOW);

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
    digitalWrite(led13, LOW);
    (button3_state = digitalRead(button3) != 1) ? button3_state = 0, button4_state = 1 : button4_state = 0;

    show_analog_value();
  }

  //stop mode if button 3 is pressed
  while (button3_state != 1) {
    motor(0, 0);
    digitalWrite(led13, HIGH);
    button_status();
  }
}

void sensor_reading() {
  sensor_position = 0;
  for (byte i = 0; i < 6; i++) {
    s[i] = analogRead(i);  //read analog value 
    if (s[i] > threshold) s[i] = 1;
    else s[i] = 0;
  }
  sensor_position = (s[0] * 1 + s[1] * 2 + s[2] * 4 + s[3] * 8 + s[4] * 16 + s[5] * 32);
  if (s[0] + s[1] + s[2] + s[3] + s[4] + s[5]) average = sensor_position / (s[0] + s[1] + s[2] + s[3] + s[4] + s[5]);  //average value
}

void PID_LINE_FOLLOW() {
  int P, I, D, PID;
  int left_speed = 250, right_speed = 250;
  int left_motor, right_motor;
  int turn_speed = 100;  //turn speed
  int setpoint = 3;
  int kp = 100, kd = 500;  //need to be adjusted

  while (1) {
    sensor_reading();
    current_error = setpoint - average;
    P = current_error * kp;
    I = 0;
    D = kd * (current_error - prev_current_error);
    PID = P + I + D;
    prev_current_error = current_error;

    right_motor = right_speed - PID;
    left_motor = left_speed + PID;
    motor(left_motor, right_motor);

    if ((s[0] + s[1] + s[2] + s[3] + s[4] + s[5]) == 0) {
      digitalWrite(led12, HIGH);
      if (turn != 's') {
        motor(0, 0);
        (turn == 'r') ? motor(-turn_speed, turn_speed), digitalWrite(led12, LOW) : motor(turn_speed, -turn_speed), digitalWrite(led12, LOW);
        while (s[2] == 0 && s[3] == 0) sensor_reading();
        turn = 's';
      }
    }

    if (s[0] == 0 && s[5] == 1) turn = 'l';
    if (s[5] == 0 && s[0] == 1) turn = 'r';

    else if ((s[0] + s[1] + s[2] + s[3] + s[4] + s[5]) == 6) {
      sensor_reading();
      if ((s[0] + s[1] + s[2] + s[3] + s[4] + s[5]) == 6) {
        motor(0, 0);
        while ((s[0] + s[1] + s[2] + s[3] + s[4] + s[5]) == 6) sensor_reading();
      } else if ((s[0] + s[1] + s[2] + s[3] + s[4] + s[5]) == 0) turn = 'r';
    }
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
