//motor pin defines
#define rmf 4 //IN1
#define rmb 7 //IN2
#define lmf 3 //IN3
#define lmb 2 //IN4
#define rms 6 //EnA
#define lms 5 //EnB

//declaration of necessary variable to line follow
int s[6], sum;
int position[6] = { 1, 2, 3, 4, 5, 6 };
int threshold = 512, sensor;
int lbase = 200, rbase = 200;
int lmotor, rmotor;
float avg;
int kp = 30, kd = 2000;
int PID;
float error, last_error;
char turn;
int base[6] = { 1, 2, 4, 8, 16, 32 };
int tsp = 100;  //turn speed

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


void loop(){
  digitalWrite(led13, HIGH);
  button_status(); //reading the status of push button

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

  PID_LINE_FOLLOW(); //line follow using pid
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
  motor(0,0);
  button_status();
}

while (button4_state != 1) {
  digitalWrite(led13, LOW);
  (button3_state = digitalRead(button3) != 1) ? button3_state = 0, button4_state = 1 : button4_state = 0;

  for (short int i = 0; i <=5 ; i++) Serial.print(String(analogRead(i)) + " "); //to print sensor value in serial monitor
  Serial.println();

  digitalWrite(led13, LOW);
  digitalWrite(led12, HIGH);
  delay(50);
  digitalWrite(led12, LOW);
  delay(50);
}

//stop mode if button 3 is pressed
while (button3_state != 1) {
  motor(0,0);
  digitalWrite(led13, HIGH);
  button_status();
}
}


void PID_reading() {
  sensor = 0;
  sum = 0;
  for (byte i = 0; i < 6; i++) {
    s[i] = analogRead(i);
    (s[i] > threshold) ? s[i] = 1 : s[i] = 0; //this line converts analog value into digital
    sensor += s[i] * position[i];
    sum += s[i];
  }
  if(sum) avg = sensor / sum; //average value
}

void reading() {
  sensor = 0;
  sum = 0;
  for (byte i = 0; i < 6; i++) {
    s[i] = analogRead(i);
    (s[i] > threshold) ? s[i] = 1 : s[i] = 0; //this line converts analog value into digital
    sensor += s[i] * base[i];
    sum += s[i];
  }
}

void PID_LINE_FOLLOW() {
  while(1) {
  reading();
  if (sum == 0) {
    digitalWrite(led12, HIGH);
    if (turn != 's') {
      delay(30);  //to turn from the center of the robot. adjust delay according to your need.
      motor(0,0);
      (turn == 'r') ? motor(-tsp, tsp), digitalWrite(led12, LOW) : motor(tsp, -tsp), digitalWrite(led12, LOW);
      while (!s[2] && !s[3]) reading();
      turn = 's';
    }
  }
  
  PID_reading();
  error = 3 - avg;
  PID = error * kp + kd * (error - last_error);
  last_error = error;

  rmotor = rbase - PID;
  lmotor = lbase + PID;
  motor(lmotor, rmotor);

  if (s[0] && !s[5]) turn = 'r';
  if (s[5] && !s[0]) turn = 'l';
  else if (sum == 6) {
    delay(20);
    reading();
    if (sum == 6) {
      motor(0, 0);
      while (sum == 6) reading();
    }
    else if (sum == 0) turn = 'r';
  }

  }
}

//reading the status of push button and saving the state.
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
  } 
  else {
    a = -(a);
    digitalWrite(lmf, 0);
    digitalWrite(lmb, 1);
  }
  if (b > 0) {
    digitalWrite(rmf, 1);
    digitalWrite(rmb, 0);
  } 
  else {
    b = -(b);
    digitalWrite(rmf, 0);
    digitalWrite(rmb, 1);
  }

  if (a > 250) a = 250;
  if (b > 250) b = 250;

  analogWrite(lms, a);
  analogWrite(rms, b);
}
