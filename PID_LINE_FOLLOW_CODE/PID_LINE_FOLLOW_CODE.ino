/*
𝐂𝐎𝐍𝐓𝐀𝐂𝐓 𝐖𝐈𝐓𝐇 𝐌𝐄:
---------------------------------------------
𝑬-𝒎𝒂𝒊𝒍: aslamhshakil20@gmail.com
𝑭𝒂𝒄𝒆𝒃𝒐𝒐𝒌: https://www.facebook.com/aslamhossain3852
𝑾𝒉𝒂𝒕'𝒔 𝑨𝒑𝒑: +𝟖𝟖𝟎𝟏𝟖𝟕𝟕-𝟓𝟒𝟑𝟖𝟐𝟓 (Only Text)
𝑻𝒆𝒍𝒆𝒈𝒓𝒂𝒎: https://t.me/+NmZDetFcTvA0MGU1

𝐁𝐔𝐘 𝐭𝐡𝐞 𝐋𝐢𝐧𝐞 𝐅𝐨𝐥𝐥𝐨𝐰𝐢𝐧𝐠 𝐑𝐨𝐛𝐨𝐭 𝐂𝐨𝐦𝐩𝐨𝐧𝐞𝐧𝐭𝐬 (𝐅𝐮𝐥𝐥 𝐊𝐢𝐭): 
From: (Ctrl+click) https://www.facebook.com/photo/?fbid=472898135830309&set=a.191125387340920
or Contact through whatsApp to Buy: +𝟖𝟖𝟎𝟏𝟖𝟕𝟕-𝟓𝟒𝟑𝟖𝟐𝟓 
Items:
1. 12x15 cm LFR Chassis x1
2. N20 Gear Motor (600RPM) x2
3.  N20 Wheels x2
4. N20 Motor Mount x2
5. N20 Ball Caster x1
6. IR Sensor Array (6 CH) x1
7. L298N Motor Driver x1
8. Buck Module x1
9. Boost Module x1
10. Arduino Nano x1
11. Toggle Switch x1
12. Female Header Pin for Arduino Connection x2
13. Push Button x2
14. 5mm LED x2
15. Resistor x2
16. T connector x1
17. 3.7 volt Li-On Battery x2

𝐍𝐨𝐭𝐞: 𝐏𝐥𝐞𝐚𝐬𝐞 𝐑𝐞𝐚𝐝 𝐭𝐡𝐞𝐬𝐞 𝐢𝐧𝐬𝐭𝐫𝐮𝐜𝐭𝐢𝐨𝐧 𝐩𝐫𝐨𝐩𝐞𝐫𝐥𝐲
______________________________________________________
1. Please Follow the circuit diagram for connection.
1. connect IR sensor pin to analog pins A0 to A5. right to left sequence.
2. like that, (from top view) the most left sensor to A5, then A4, A3, A2, A1, and the right most sensor pin to A0
3. count the sensor pin from top view of the robot.
4. First take the analog reading of the sensor.
5. Note the sensors analog value from white and black surface.
4. This code is for sensors which gives maximum valaue in black surface and minimum in white.
If your sensor gives maximum value in white and minimum in black then you need to change
a little bit in analog to digital conversion in line number 220 and 221 of this code.
go there, i have given the necessary instruction.
6. you must assigne theshold value for all the sensor properly.
in line number 72 threshold[sensorNumber] = { 300, 300, 300, 300, 300, 300 };
calculate threshold value by this equation, threshold = (maximum analog value + minimum analog value) / 2.
analog value can be vary from different track.
normally the printed track provides analog value on black surface is around 500-700.
where black duck tap provides 900+.
7. Adjust Kp Only from 50-90
8. this is a basic pid code. not suitable for competition where maze type like is seen.
so you need to upgrade the code for competition.
you can take assistance from me.
*/

//Motor pins (if you fail to connect them correctly then robot will not run)
#define rmf 7  //IN1 (Right Motor Forward)
#define rmb 4  //IN2 (Right Motor backward)
#define lmf 3  //IN3 (left Motor Forward)
#define lmb 2  //IN4 (left Motor backward)
//EnA and EnB
#define rms 6  //EnA (Right Motor Speed)
#define lms 5  //EnB (Left Motor Speed)

//Defines button pins connected to arduino
#define button1 8
#define button2 9
#define button3 10
#define button4 11

//Defines LED pins connected to arduino
#define led13 13
#define led12 12

#define sensorNumber 6  //number of sensor in the array
int sensor[sensorNumber];

//threshold need to be adjusted according to analog value in black surface (maximum) and white surface (minimum).
//calculate threshold = (max analog value + min analog value) / 2.
int threshold[sensorNumber] = { 300, 300, 300, 300, 300, 300 };

//Necesssary Delays need to be adjusted
#define turn_delay 10    //increase if the robot turns too early. and reduce if the it goes too far from the line before turn
#define u_turn_delay 50  //increase if the robot turns too early. and reduce if the it goes too far from the line before turn
#define stop_timer 30    //reduce if the robot doesn't stop in stop section. this value always should be more than 20

//Necessary Variables
int max_speed = 250;          //adjustable
int left_motor_speed = 200;   //adjustable. left_motor_speed and right_motor_speed should be same
int right_motor_speed = 200;  //adjustable
int turn_speed = 150;         //adjustable from 80-120
int sensor_sum;
float calculated_pos;

int kp = 70;  //need to be adjusted.

/*Increasing 𝐾𝑝 (Proportional Gain):

Effect: The robot will react more aggressively to deviations from the line.
Faster correction of errors.
More responsive to line changes.
If too high, it can cause oscillations, making the robot unstable.
Might overshoot the line frequently.

Decreasing 𝐾𝑝
Effect: The robot will react more slowly to deviations.
Smoother movement with less oscillation.
Slower response, making it struggle with sharp turns.
May not correct deviations effectively, causing the robot to drift off the line.
*/

int kd = 1000;  //need to be adjusted

/*
Increasing 𝐾𝑑 (Derivative Gain):
Effect: The robot anticipates future errors by considering the rate of change of the error.
Helps dampen oscillations caused by high 𝐾𝑝​.
Improves stability and smoothness.
If too high, the robot may become too cautious and slow down excessively.
Can amplify noise from sensors, leading to jittery movement.

Decreasing 𝐾𝑑:
Effect: The robot will have less damping and might oscillate more.
The robot will respond faster to errors.
Increased oscillations, especially with high 𝐾𝑝.
Can make the movement unstable, especially at high speeds.

*/

int PID;
int turn_value = 0;
float error, previous_error;
float center_point = 3.5;
int line_position;
bool button1_state, button2_state, button3_state, button4_state;



void setup() {
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);

  pinMode(rms, OUTPUT);
  pinMode(lms, OUTPUT);

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);

  pinMode(led13, OUTPUT);
  pinMode(led12, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  button_status();  //sensor_reading the status of push button

  //when button 1 will be pressed then the robot will start to follow lines.
  while (button1_state != 1) {
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

    (button3_state = digitalRead(button3) != 1) ? button3_state = 0, button4_state = 1 : button4_state = 0;

    display_analog_value();
  }

  //stop mode if button 3 is pressed
  while (button3_state != 1) {
    motor(0, 0);
    digitalWrite(led12, HIGH);
    button_status();
  }
}


//sensor_reading the status of push button and saving the state.
void button_status() {
  button1_state = digitalRead(button1);
  button2_state = digitalRead(button2);
  button3_state = digitalRead(button3);
  button4_state = digitalRead(button4);
}

//*************************pid sensor reading****************************************
void read_sensor() {
  line_position = 0;
  sensor_sum = 0;

  for (byte i = 0; i < sensorNumber; i++) {
    sensor[i] = analogRead(i);
    if (sensor[i] > threshold[i]) sensor[i] = 1;  //if your sensor gives higest (>900) analog value in white then replce it with 0
    else sensor[i] = 0;                           //if your sensor gives lowest analog value (50-100) in black then replce it with 1
    //Example:
    //if (sensor[i] > threshold[i]) sensor[i] = 0; //uncomment 223 and 224 Number line
    //else sensor[i] = 1;                         //and make comment 220 and 221 Number line
  }

  sensor_sum = (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5]);
  line_position = (sensor[0] * 1 + sensor[1] * 2 + sensor[2] * 3 + sensor[3] * 4 + sensor[4] * 5 + sensor[5] * 6);
  if (sensor_sum) calculated_pos = line_position / sensor_sum;
}

//***********************Display Analog Value in Serial Monitor******************
void display_analog_value() {
  while (1) {
    for (int i = 5; i >= 0; i--) {
      sensor[i] = analogRead(i);
      Serial.print(String(sensor[i]) + " ");
    }
    Serial.println();

    digitalWrite(led12, LOW);
    digitalWrite(led13, HIGH);
    delay(50);
    digitalWrite(led13, LOW);
    delay(50);
  }
}

//****************************Digital Value in Serial Moitor*******************************
void digital_reading() {
  while (1) {
    read_sensor();
    for (int i = 5; i >= 0; i--)
      Serial.print(String(sensor[i]) + " ");

    digitalWrite(led12, LOW);
    digitalWrite(led13, HIGH);
    delay(50);
    digitalWrite(led13, LOW);
    delay(50);
  }
}


//*******************************motor control function**************************************
void motor(int left, int right) {
  if (right > 0) {
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
  } else {
    right = -(right);
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, HIGH);
  }
  if (left > 0) {
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
  } else {
    left = -(left);
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
  }

  if (left > max_speed) left = max_speed;
  if (right > max_speed) right = max_speed;

  analogWrite(lms, left);
  analogWrite(rms, right);
}

//***********************************Motor Check Function************************************
void motor_check() {
  motor(250, 250);
  delay(3000);
  motor(0, 0);
}

//****************************PID_LINE_FOLLOW function****************************************
void Line_Follow() {
  while (1) {
    read_sensor();

    error = center_point - calculated_pos;
    PID = error * kp + kd * (error - previous_error);
    previous_error = error;

    int right_motor = right_motor_speed - PID;
    int left_motor = left_motor_speed + PID;

    motor(left_motor, right_motor);  //Straight LINE

    // left Turn Detection
    if (sensor[5] == 1 && sensor[0] == 0) turn_value = 1;

    //right turn detection
    if (sensor[5] == 0 && sensor[0] == 1) turn_value = 2;




    //Turn Execution when all the sensor in white
    if (sensor_sum == 0) {

      //Left turn execution
      if (turn_value == 1) {  //check turn value
        delay(turn_delay);
        motor(-turn_speed, turn_speed);
        while (sensor[3] == 0 && sensor[2] == 0) read_sensor();
        turn_value = 0;
      }

      //Right Turn execution
      else if (turn_value == 2) {
        delay(turn_delay);
        motor(turn_speed, -turn_speed);
        while (sensor[3] == 0 && sensor[2] == 0) read_sensor();
        turn_value = 0;
      }
      //U-Turn execution
      else if (turn_value == 0) {
        delay(u_turn_delay);
        motor(turn_speed, -turn_speed);
        while (sensor[3] == 0 && sensor[2] == 0) read_sensor();
        turn_value = 0;
      }
    }

    //when all the sensor in black (end, cross, T section)
    else if (sensor_sum == 6) {
      delay(stop_timer);
      read_sensor();
      if (sensor_sum == 6) {
        motor(0, 0);  //STOP
        while (sensor_sum == 6) read_sensor();
      } else if (sensor_sum == 0) turn_value = 2;  //go right in T section
    }
  }
}
