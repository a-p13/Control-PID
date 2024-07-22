//PID control for position closed loop implementation
//Design of control systems
//15 - Oct - 2023


//Pin definitions
const int encoderPinA = 2;
const int encoderPinB = 3;

const int motorDirA = 13; //direction control 1
const int motorDirB = 12; //direction control 2
const int ENpin = 11; //PWM input
///////////////////////////////////////////////////////
//Distances and physical model variables
const float motorPpr = 341.2; //Motor encoder pulses per revolution
const int mmPerRev = 8; 
long currentPosPulses = 0; //saves the current position in pulses;
bool encoderDirection = true; //true for forward movement, false for backwards movement

long int encoderPosPulses = 0; //Current encoder pulse counting
float  currentPosInRads = 0; //The current position in terms of radians
float  setPointInRad = 0; //Desired position set point in radians
uint8_t currentPWM = 0; //PWM var, form 0 to 255; 
int motorDirection = 0; //0 for stop, 1 for forward direction and -1 for backwards direction

//Auxiliary vars
float setPoint = 0.0;
float error = 0.0;
float prevErrors [2] = {0.0, 0.0};
uint8_t prevPWM = 0;
float desiredPosInMM = 0.0;
float a = 0.0; //Auxiliary variable for comparing the encoder pins
float b = 0.0; //Auxiliary variable for comparing the encoder pins

//PID implementation VARS
float kp = 4.1;
float ki = 25;
float kd = 0;
float T_s = 0.02; // Tiempo respuesta

//ecs. from page 370 lyshevsky


void setup() {
  Serial.begin(9600);

  //Initialization of all vars
  attachInterrupt(digitalPinToInterrupt(2), encoderRead, RISING);
  pinMode(motorDirA, OUTPUT);
  pinMode(motorDirB, OUTPUT);
  pinMode(ENpin, OUTPUT);
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    String stringVal = Serial.readString();
    desiredPosInMM = stringVal.toInt();
  }
  //Serial.print("\n");
  Serial.print(" Error: ");
  Serial.print(error);
  //Serial.print("\n");
  Serial.print(" RadsP : ");
  Serial.println(currentPosInRads);
  //Serial.print("\n");
  Serial.print(" SetP : ");
  Serial.print(setPoint);
  //Serial.print("\n");
  Serial.print(" DirMotor : ");
  Serial.print(motorDirection);
  //Serial.print("\n");
  PIDimplementation();


}


void encoderRead(){
  a = digitalRead(encoderPinA);
  b = digitalRead(encoderPinB);
  
  if(a =! b){ //diferentes
    encoderPosPulses --;
    encoderDirection = false;
  }
  else{  //iguales
    encoderPosPulses ++;
    encoderDirection = true;
  }
}

void motorControlSignals(uint8_t pwm, int motorDir){
  if(motorDir == 1){
    digitalWrite(motorDirA, LOW);
    digitalWrite(motorDirB, HIGH);
  }
  else if (motorDir == -1){
    digitalWrite(motorDirA, HIGH);
    digitalWrite(motorDirB, LOW);
  }
  else{
    digitalWrite(motorDirA, LOW);
    digitalWrite(motorDirB, LOW);
  }
  analogWrite(ENpin, pwm);
}

void PIDimplementation(){
  error = setPoint-currentPosInRads;

  prevErrors[1]=prevErrors[0];  
  prevErrors[0]=error;
  prevPWM = currentPWM;
  float k_e0 = kp + (0.5 * ki * T_s) + (2*(kd/T_s));
  float k_e1 = ki*T_s-(4*(kd/T_s));
  float k_e2 = -kp + (0.5*ki*T_s) + 2*(kd/T_s);

  currentPosInRads = 2*encoderPosPulses * ((2*3.1416)/motorPpr);
  setPoint = desiredPosInMM*((2*3.1416)/8);
  

  currentPWM = prevPWM + k_e0*error + k_e1*prevErrors[0] + k_e2*prevErrors[1];
// -2**-1***0**1***1
  if(error < -0.8){
    motorDirection = -1;
  }
  else if (error > 0.8){
    motorDirection = 1;
  }
  else{
    currentPWM = 0;
    motorDirection = 0;
  }
  //Serial.println(" Current pwm: " + String(currentPWM));

  motorControlSignals(currentPWM, motorDirection);

}
