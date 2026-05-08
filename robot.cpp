#include <Ultrasonic.h>

// Left color sensor
#define LS3 14
#define LS2 15
#define LOUT 22
#define LS0 23
#define LS1 18

// Right color sensor
#define RS3 19
#define RS2 20
#define ROUT 21
#define RS0 17
#define RS1 16

#define ena 5
#define in1 6
#define in2 7
#define in3 9
#define in4 10
#define enb 11

#define LS 4
#define MS 3
#define RS 2

#define front_trigPin 40
#define front_echoPin 41

#define side_trigPin 13
#define side_echoPin 12

Ultrasonic ultrasonic_front(front_trigPin, front_echoPin);
Ultrasonic ultrasonic_side(side_trigPin, side_echoPin);

const int object_detection_distance = 12; // cm

int object_evation_timer = 0;
const int object_evation_time = 200;

int LgreenFrequency = 0;
int RgreenFrequency = 0;

int speed = 180;

void setup()
{

  digitalWrite(LS0, HIGH);
  digitalWrite(LS1, LOW);

  digitalWrite(RS0, HIGH);
  digitalWrite(RS1, LOW);

  // put your setup code here, to run once:
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(LS, INPUT);
  pinMode(MS, INPUT);
  pinMode(RS, INPUT);

  Serial.begin(9600);
}

void forward()
{

  // Serial.print("forward");

  analogWrite(ena, speed);
  analogWrite(enb, speed);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void sharp_right()
{

  // Serial.print("sharp right");

  analogWrite(ena, 255);
  analogWrite(enb, speed);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void right()
{

  // Serial.print("right");

  analogWrite(ena, 255);
  analogWrite(enb, speed + 30);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void left()
{

  // Serial.print("left");

  analogWrite(ena, speed + 30);
  analogWrite(enb, 255);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void sharp_left()
{

  // Serial.print("sharp left");

  analogWrite(ena, speed);
  analogWrite(enb, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void stop()
{
  analogWrite(ena, 0);
  analogWrite(enb, 0);
}

void obstacle()
{
  digitalWrite(13, HIGH);
  stop();
  delay(1000);

  int distance = 357;

  left();
  ;
  distance = ultrasonic_side.read();
  if (distance == 0)
  {
    distance = 357;
  }

  distance = 357;
  while (digitalRead(LS) == HIGH && digitalRead(MS) == HIGH && digitalRead(RS) == HIGH)
  {
    if (millis() < object_evation_timer + object_evation_time)
    {
      forward();
      digitalWrite(13, HIGH);
      Serial.println(millis());
    }
    else
    {
      object_evation_timer = millis();
      Serial.print(" heheh ");
      Serial.println(object_evation_timer);

      distance = ultrasonic_side.read();
      if (distance == 0)
      {
        distance = 357;
      }

      Serial.println(distance);

      while (distance > object_detection_distance + 30)
      {
        digitalWrite(13, LOW);
        sharp_right();
        distance = ultrasonic_side.read();
        if (distance == 0)
        {
          distance = 357;
        }
        Serial.println(distance);
      }
      delay(100); // to let it finish a bit more turning
    }
  }
  left();
}

void loop()
{

  /*
    Serial.print(" ");
    Serial.print(digitalRead(LS));
    Serial.print(" ");
    Serial.print(digitalRead(MS));
    Serial.print(" ");
    Serial.print(digitalRead(RS));
    Serial.print(" ");
    */

  node();
}

void node()
{
  if (digitalRead(LS) == HIGH && digitalRead(MS) == LOW && digitalRead(RS) == HIGH)
  {
    forward();
    digitalWrite(13, HIGH);
  }

  if (digitalRead(LS) == HIGH && digitalRead(RS) == LOW)
  {
    right();
  }

  if (digitalRead(LS) == LOW && digitalRead(RS) == HIGH)
  {
    left();
  }
  if (digitalRead(LS) == LOW && digitalRead(MS) == LOW && digitalRead(RS) == LOW)
  {
    stop();
    // 2. Read LGreeUn Photodiodes
    digitalWrite(LS2, HIGH); // select green filter
    digitalWrite(LS3, HIGH);
    LgreenFrequency = pulseIn(LOUT, LOW, 30000);
    Serial.print("LG= ");
    Serial.print(LgreenFrequency);
    Serial.print("  ");

    // 2. Read RGreen Photodiodes
    digitalWrite(RS2, HIGH);
    digitalWrite(RS3, HIGH);
    RgreenFrequency = pulseIn(ROUT, LOW, 30000);
    Serial.print("RG= ");
    Serial.print(RgreenFrequency);
    Serial.println("  ");

    if (RgreenFrequency > LgreenFrequency)
    {
      Serial.println("right");
      right();
    }
    else
    {
      Serial.println("left");
      left();
    }

    if (RgreenFrequency > 25 && RgreenFrequency < 40)
    {
      analogWrite(ena, speed);
      analogWrite(enb, speed);

      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);

      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      delay(1000);
    }
  }

  int distance = ultrasonic_front.read();
  if (distance == 0)
  {
    distance = 357;
  }
  if (distance < object_detection_distance)
  {
    obstacle();
  }
}