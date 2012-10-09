// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo1;  // create servo object to control a servo 
Servo myservo2;  // create servo object to control a servo 
Servo myservo3;  // create servo object to control a servo 
/*
Servo myservo4;  // create servo object to control a servo 
Servo myservo5;  // create servo object to control a servo 
Servo myservo6;  // create servo object to control a servo 
Servo myservo7;  // create servo object to control a servo 
Servo myservo8;  // create servo object to control a servo 
Servo myservo9;  // create servo object to control a servo 
Servo myservo10;  // create servo object to control a servo 
Servo myservo11;  // create servo object to control a servo 
Servo myservo12;  // create servo object to control a servo 
*/
 
int pos = 0;    // variable to store the servo position 
float ground=-100;
float ceiling=-50;

boolean s=false;

void setup() 
{
  pinMode(34,OUTPUT); 
  myservo1.attach(22);  // attaches the servo on pin 9 to the servo object 
  myservo2.attach(23);  // attaches the servo on pin 9 to the servo object 
  myservo3.attach(24);  // attaches the servo on pin 9 to the servo object 
  /*
  myservo4.attach(25);  // attaches the servo on pin 9 to the servo object 
  myservo5.attach(26);  // attaches the servo on pin 9 to the servo object 
  myservo6.attach(27);  // attaches the servo on pin 9 to the servo object 
  myservo7.attach(28);  // attaches the servo on pin 9 to the servo object 
  myservo8.attach(29);  // attaches the servo on pin 9 to the servo object 
  myservo9.attach(30);  // attaches the servo on pin 9 to the servo object 
  myservo10.attach(31);  // attaches the servo on pin 9 to the servo object 
  myservo11.attach(32);  // attaches the servo on pin 9 to the servo object 
  myservo12.attach(33);  // attaches the servo on pin 9 to the servo object 
  */
  nil_servos();
} 
 
 
void loop() 
{
  s=!s;
  digitalWrite(34,s);
  nil_servos();
  delay(50);
  return;
  for(int x=-38;x<38;x++)
  {
    go_to_xyz(x,0,ground);
    delay(10);                       // waits 15ms for the servo to reach the position 
  }
  go_to_xyz(38,0,ceiling);
  delay(500);
  go_to_xyz(-38,0,ceiling);
  delay(500);
  go_to_xyz(-38,0,ground);
  delay(500);

}
void nil_servos()
{
  myservo1.write(1);              // tell servo to go to position in variable 'pos' 
  myservo2.write(1);              // tell servo to go to position in variable 'pos' 
  myservo3.write(1);              // tell servo to go to position in variable 'pos'
  /*myservo4.write(0);              // tell servo to go to position in variable 'pos' 
  myservo5.write(0);              // tell servo to go to position in variable 'pos' 
  myservo6.write(0);              // tell servo to go to position in variable 'pos'
  myservo7.write(0);              // tell servo to go to position in variable 'pos' 
  myservo8.write(0);              // tell servo to go to position in variable 'pos' 
  myservo9.write(0);              // tell servo to go to position in variable 'pos'
  myservo10.write(0);              // tell servo to go to position in variable 'pos' 
  myservo11.write(0);              // tell servo to go to position in variable 'pos' 
  myservo12.write(0);              // tell servo to go to position in variable 'pos'
*/
}
void go_to_xyz(float x,float y, float z)
{
  float t1,t2,t3;
  delta_calcInverse(x,y,z,t1,t2,t3);
  myservo1.write(t1);              // tell servo to go to position in variable 'pos' 
  myservo2.write(t2);              // tell servo to go to position in variable 'pos' 
  myservo3.write(t3);              // tell servo to go to position in variable 'pos'
    
}
 // robot geometry
 // (look at pics above for explanation)
 const float e = 1.0;     // end effector
 const float f = 22.3;     // base
 const float re = 85.0;
 const float rf = 60.0;
 
 // trigonometric constants
 const float sqrt3 = sqrt(3.0);
 const float pi = 3.141592653;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1/sqrt3;
 
 // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
 }
 
 // inverse kinematics
 // helper functions, calculates angle theta1 (for YZ-pane)
 int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // shift center to edge
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // non-existing point
     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
     return 0;
 }
 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     return status;
 }
