







#include<Servo.h>
#include<PID_v1.h>


const int servoPin = 9;                                               //Servo Pin

float Kp = 2.5;                                                    //Initial Proportional Gain
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 1.1;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput, value;



PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.




Servo myServo;                                                       //Initialize Servo.


void setup() {

  Serial.begin(9600);                                                //Begin Serial
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
  //  position as the input to the PID algorithm



  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC
  myPID.SetOutputLimits(-80, 80);                                    //Set Output limits to -80 and 80 degrees.
}

void loop()
{
  Setpoint = 20;
  Input = readPosition();

  myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees

  ServoOutput = 90 + Output;                                        //  είναι ήδη στραμμένο κατά 90
  myServo.write(ServoOutput);                                        //Writes value of Output to servo


}




float readPosition() {
  delay(40);                                                            //Don't set too low or echos will run into eachother.


  const int pingPin = 7;
  const int echoPin = 8;

  long duration, cm;
  unsigned long now = millis();                                        //Για να διαβαστεί ξεκάθαρα ο παλμός
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);


  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  cm = duration / (29*2);


  if (cm > 34)    // για τον θόρυβο
  {
    cm = 34;
  }

 

  Serial.println(cm);

  return cm;                                          //Returns distance value.
}



