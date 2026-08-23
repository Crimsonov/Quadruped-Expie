// ----- LIBRARIES -----
#include <Wire.h> 
#include <Adafruit_PWMServoDriver.h> 

// ----- ADDRESS -----
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

// ----- SERVO ID VARIABLES -----
// Servo number that corresponds with PCA9685 pin 

// Front Left Leg
const int SERVO_1 = 0;
const int SERVO_2 = 1;
const int SERVO_3 = 2;

// Front Right Leg
const int SERVO_4 = 3;
const int SERVO_5 = 4;
const int SERVO_6 = 5;

// Back Right Leg
const int SERVO_7 = 6;
const int SERVO_8 = 7;
const int SERVO_9 = 8;

// Back Left Leg
const int SERVO_10 = 9;
const int SERVO_11 = 10;
const int SERVO_12 = 11;

// ----- LEG NUMBER ID -----
const int topleft = 1;
const int topright = 2;
const int bottomright = 3;
const int bottomleft = 4;
const int legnumber[4] = {topleft, topright, bottomright, bottomleft};

// ----- OFFSET -----
// Servo_# offset from left to right
const int offset[12] = {0, 25, 0, -25, 0, 0, 0, 25, 0, 0, 0, -25};

// ----- DOUBLE MAP MATH FUNCTION -----
// !!! Built-in arduino map() function outputs INTEGERS !!!
double doubleMap(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ----- DISTANCE MATH FUNCTION -----
double distance(double x1, double y1, double x2, double y2){
  return (sqrt(sq(x2 - x1) + sq(y2 - y1)));
}

// ----- KEY COORDINATES -----
// Total key coordinates
const int cordArray = 3;

// Given coordinates are in the unit of mm
double FL_Xcord[cordArray] = {0, 0, 0};
double FL_Zcord[cordArray] = {-169.71, -119.71, -169.71};

double FR_Xcord[cordArray] = {0, 0, 0};
double FR_Zcord[cordArray] = {-169.71, -119.71, -169.71};

double BR_Xcord[cordArray] = {0, 0, 0};
double BR_Zcord[cordArray] = {-169.71, -119.71, -169.71};

double BL_Xcord[cordArray] = {0, 0, 0};
double BL_Zcord[cordArray] = {-169.71, -119.71, -169.71};

// ----- INCREMENT CALCULATOR -----
// increment * total_points = total_distance
// More points result in slower movements while Less points result in faster movements
const int totalpoints = 50; 

double incrementCalc(double oldXcord, double oldZcord, double newXcord, double newZcord){
  
  double increment = distance(oldXcord, oldZcord, newXcord, newZcord) / totalpoints;
  
  // Strict Horizontal or Vertical increments
  // A Xcord or a Zcord must have the same value from old to new 
  if(oldXcord == newXcord || oldZcord == newZcord){
    
    // Positive increments 
    if(oldXcord < newXcord || oldZcord < newZcord){
      return increment;
    }

    // Negative increments 
    if(oldXcord > newXcord || oldZcord > newZcord){
      return increment * -1;
    }

  }

  // Increment result if a slope is involved
  else{
    return increment;
  }

}

// ----- COORDINATE INCREMENT -----
void cordInc(int legID, int incrementNumb, double oldXcord, double oldZcord, double newXcord, double newZcord){

  // Old coordinates and new coordinates forming a line with no slope
  if(oldXcord == newXcord || oldZcord == newZcord){
    
    // Changing Xcord
    if(oldXcord < newXcord || oldXcord > newXcord){
      double incrementCord = (incrementNumb * incrementCalc(oldXcord, oldZcord, newXcord, newZcord)) + oldXcord;
      legInverseKinematics(legID, incrementCord, newZcord);
      return;
    }

    // Changing Zcord
    if(oldZcord < newZcord || oldZcord > newZcord){
      double incrementCord = (incrementNumb * incrementCalc(oldXcord, oldZcord, newXcord, newZcord)) + oldZcord;
      legInverseKinematics(legID, newXcord, incrementCord);
      return;
    }

    // Non changing Xcord and Zcord 
    // Servos keep last position
    if(oldXcord == newXcord && oldZcord == newZcord){
      return;
    }

  }

  // Old coordinates and new coordinates forming a line with a slope
  else{
    // Coordinates can form a right triangle 
    // Hypotenuse: Distance of (oldX, oldZ) and (newX, newZ) 
    // Side Length: Distance from oldX to newX
    // Result: angleX and angleZ
    double XcordLength = distance(oldXcord, oldZcord, newXcord, oldZcord);
    double angleZrads = acos(XcordLength / distance(oldXcord, oldZcord, newXcord, newZcord));
    double angleZdegs = angleZrads * RAD_TO_DEG;
    double angleXdegs = 90 - angleZdegs; // All triangles are 180 degrees 

    // New hypotenuse distance the reaches the incrementNumb point
    // Ex: 3 = point#2 * 1.5mm 
    double incrementCord = incrementNumb * incrementCalc(oldXcord, oldZcord, newXcord, newZcord);
    
    // The original larger triangle is simillar to the increment triangle
    // angleX and angleZ can be used to find the needed horizontal and vertical increments needed
    double x_incrementrads = angleZdegs * DEG_TO_RAD;
    double x_increment = cos(x_incrementrads) * incrementCord; 
    double z_incrementrads = angleXdegs * DEG_TO_RAD;
    double z_increment = cos(z_incrementrads) * incrementCord;

    double Xcord;
    double Zcord;

    // Decreasing X and Decreasing Z
    if(oldXcord > newXcord && oldZcord > newZcord){
      Xcord = oldXcord - x_increment;
      Zcord = oldZcord - z_increment;
      legInverseKinematics(legID, Xcord, Zcord);
    }

    // Increasing X and Decreasing Z
    if(oldXcord < newXcord && oldZcord > newZcord){
      Xcord = oldXcord + x_increment;
      Zcord = oldZcord - z_increment;
      legInverseKinematics(legID, Xcord, Zcord);
    }

    // Increasing X and Increasing Z
    if(oldXcord < newXcord && oldZcord < newZcord){
      Xcord = oldXcord + x_increment;
      Zcord = oldZcord + z_increment;
      legInverseKinematics(legID, Xcord, Zcord);
    }

    // Decreasing X and Increasing Z
    if(oldXcord > newXcord && oldZcord < newZcord){
      Xcord = oldXcord - x_increment;
      Zcord = oldZcord + z_increment;
      legInverseKinematics(legID, Xcord, Zcord);
    }

  }
  
}

// ----- INVERSE KINEMATICS -----
void legInverseKinematics(int legID, double Xcord, double Zcord){
  // 2D Inverse Kinematics
  // Ycord is not included 

  // Joint Lengths
  const double J1 = 54; // Not used 
  const double J2 = 120;
  const double J3 = 120;

  // Total displacement from the origin to end effector
  double displacement = sqrt(sq(Xcord) + sq(Zcord));

  // Finds what angle joint 3 makes 
  double angleJ3rads = acos((sq(J3) - sq(J2) - sq(displacement)) / (-2 * J2 * displacement)); 
  double angleJ3degs = abs(angleJ3rads * RAD_TO_DEG);
  double THETA2;

  // THETA3 result
  double angleDisplacementrads = acos((sq(displacement) - sq(J2) - sq(J3)) / (-2 * J2 * J3)); 
  double angleDisplacementdegs = abs(angleDisplacementrads * RAD_TO_DEG);
  double THETA3 = doubleMap(angleDisplacementdegs, 45, 135, 135, 45);
  THETA3 = constrain(THETA3, 35, 145);

  if (Xcord == 0){
    // Xcord being 0
    // Coordinates don't form a second triangle 
    THETA2 = doubleMap(angleJ3degs + 90, 45, 225, 0, 180); 
    THETA2 = constrain(THETA2, 0, 180);
    servoPWM(legID, THETA2, THETA3);
  }

  if (Xcord > 0) {
    // Xcord positive values 
    // atan fromed by Zcord being opposite and Xcord being adjacent 
    double angleZrads = atan(Zcord / Xcord); 
    double angleZdegs = abs(angleZrads * RAD_TO_DEG);
    
    THETA2 = doubleMap(angleJ3degs + angleZdegs, 45, 225, 0, 180);
    THETA2 = constrain(THETA2, 0, 180);
    servoPWM(legID, THETA2, THETA3);
  }

  
  if (Xcord < 0) {
    // Xcord negative values
    // atan fromed by Xcord being opposite and Zcord being adjacent 
    double angleZrads = atan(Xcord / Zcord); 
    double angleZdegs = abs(angleZrads * RAD_TO_DEG);
    
    THETA2 = doubleMap(angleJ3degs + angleZdegs + 90, 45, 225, 0, 180);
    THETA2 = constrain(THETA2, 0, 180);
    servoPWM(legID, THETA2, THETA3);
  }

}

// ----- LEG ID -----
// Referenced leg is topleft leg
void servoPWM(int legID, double THETA2, double THETA3){

  switch(legID) {

    // Front Left Leg
    case 1: {
      double MappedTheta_2 = doubleMap(THETA2, 0, 180, 500, 2500);
      double MappedTheta_3 = doubleMap(THETA3, 0, 180, 500, 2500);
      pwm.writeMicroseconds(SERVO_2, MappedTheta_2 + offset[SERVO_2]);
      pwm.writeMicroseconds(SERVO_3, MappedTheta_3 + offset[SERVO_3]);
      break;
    }

    // Front Right Leg
    case 2: {
      double MappedTheta_2 = doubleMap(THETA2, 180, 0, 500, 2500);
      double MappedTheta_3 = doubleMap(THETA3, 180, 0, 500, 2500);
      pwm.writeMicroseconds(SERVO_5, MappedTheta_2 + offset[SERVO_5]);
      pwm.writeMicroseconds(SERVO_6, MappedTheta_3 + offset[SERVO_6]);
      break;
    }

    // Back Right Leg
    case 3: {
      double MappedTheta_2 = doubleMap(THETA2, 180, 0, 500, 2500);
      double MappedTheta_3 = doubleMap(THETA3, 180, 0, 500, 2500);
      pwm.writeMicroseconds(SERVO_8, MappedTheta_2 + offset[SERVO_8]);
      pwm.writeMicroseconds(SERVO_9, MappedTheta_3 + offset[SERVO_9]);
      break;
    }

    // Back Left Leg
    case 4: {
      double MappedTheta_2 = doubleMap(THETA2, 0, 180, 500, 2500);
      double MappedTheta_3 = doubleMap(THETA3, 0, 180, 500, 2500);
      pwm.writeMicroseconds(SERVO_11, MappedTheta_2 + offset[SERVO_11]);
      pwm.writeMicroseconds(SERVO_12, MappedTheta_3 + offset[SERVO_12]);
      break;
    }

    default:
    return;
  }

}

void pushups(){
  
  for(int i = 0; i < (cordArray - 1); i++){
    for(int j = 0; j <= totalpoints; j++){
      cordInc(legnumber[0], j, FL_Xcord[i], FL_Zcord[i], FL_Xcord[i + 1], FL_Zcord[i + 1]);
      cordInc(legnumber[1], j, FR_Xcord[i], FR_Zcord[i], FR_Xcord[i + 1], FR_Zcord[i + 1]);
      cordInc(legnumber[2], j, BR_Xcord[i], BR_Zcord[i], BR_Xcord[i + 1], BR_Zcord[i + 1]);
      cordInc(legnumber[3], j, BL_Xcord[i], BL_Zcord[i], BL_Xcord[i + 1], BL_Zcord[i + 1]);
    }

  }

}

// ----- INTIALIZATION -----
void setup() {
  Serial.begin(9600);
  pwm.begin(); 
  pwm.setPWMFreq(60); // MG996R operating frequency

  // Setting hip servos to 90 degrees 
  pwm.writeMicroseconds(SERVO_1, 1500 + offset[SERVO_1]);
  pwm.writeMicroseconds(SERVO_4, 1500 + offset[SERVO_4]);
  pwm.writeMicroseconds(SERVO_7, 1500 + offset[SERVO_7]);
  pwm.writeMicroseconds(SERVO_10, 1500 + offset[SERVO_10]);

  delay(1000);

}

void loop() {
  pushups();
  
}