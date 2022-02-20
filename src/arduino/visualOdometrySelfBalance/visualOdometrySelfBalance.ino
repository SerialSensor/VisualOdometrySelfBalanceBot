
//---------------------------------------------------------------//
//
//          @@@@@@@                          @@@              ,@@@
//       @@@@*   @@@@*                       @@@              ,@@@
//       @@@       @@@    @@@@@@@    @@@@@@@ @@@   @@@@@@@@   ,@@@
//        @@@@@@@@@*    @@@     @@@  @@@@    @@@ /@@@    @@@  ,@@@
//               @@@@@ &@@@@@@@@@@@# @@@     @@@   @@@@@@@@@  ,@@@
//      ,@@@       @@@  @@@     %%%  @@@     @@@ @@@     @@@  ,@@@
//        @@@@@@@@@@@    @@@@@@@@@@  @@@     @@@ #@@@@@@@@@@  ,@@@
//
//
//  @@@@@@@@@
//@@@@     @@@@
//@@@@            ,@@@@@@@@   @@@@@@@@@@   @@@@@@@@@   %@@@@@@@@   @@@@@@@
//  @@@@@@@@@    @@@     @@@  @@@     @@@ @@@    @@@. @@@     @@@  @@@
//         (@@@  @@@@@@@@@@@@ @@@     @@@  %@@@@@@@@  @@@     *@@@ @@@
//@@@       @@@* @@@     @@@/ @@@     @@@ @@@     @@@ @@@     @@@  @@@
// @@@@@@@@@@@    @@@@@@@@@.  @@@     @@@  @@@@@@@@@   @@@@@@@@@   @@@
//
//          Example code for world's first visual odometry
//          stabilized self balance robot
// * You have to adapt this code to your motordriver / pin config
// * In general there should be plenty of room for improvements :-)
//--------------------------------------------------------------//

//--------------------------------------------------------------//
// Hardware config for DRV 8835 motor driver
//--------------------------------------------------------------//
const int PWM_PIN_RIGHT = 3;
const int DIRECTION_PIN_RIGHT = 4;
const int PWM_PIN_LEFT = 5;
const int DIRECTION_PIN_LEFT = 6;
const int LOGIC_VCC_PIN = 2;
const int MODE_PIN = 7;

//--------------------------------------------------------------//
// Sensor ID's from SerialSensor details view
//--------------------------------------------------------------//
const byte ORIENTATION_SENSOR = 99;
const byte LINE_SENSOR = 100;
const byte POSITION_SENSOR = 101;
const byte PARAMETER_SENSOR = 103;

//--------------------------------------------------------------//
// Quanitization of parameter g_receivedInput
// E.g. floatValueYouWant = receivedByte * quantization
//--------------------------------------------------------------//
const float TILT_CONTROLLER_P_QUANTIZATION = 0.05F;
const float TILT_CONTROLLER_I_QUANTIZATION = 0.1F;
const float TILT_CONTROLLER_D_QUANTIZATION = 5e-4F;
const float POS_CONTROLLER_QUANTIZATION = 0.05F;
const float YAW_CONTROLLER_QUANTIZATION = 0.05F;
const float TILT_ANGLE_QUANTIZATION = 0.003491F;
//--------------------------------------------------------------//
//Other constants & definitions
//--------------------------------------------------------------//
const float MICRO2SECONDS = 1.F / 1.0e6F;
const float DEG2RAD = 3.1416F / 180.F;
const int   MAX_PWM = 255;
const int   FRAME_LENGTH = 13;
const int   BYTES_PER_FLOAT = 4;
const int   BYTES_PER_POSITION_VALUE = 3;
const int   POSITION_DATA_LENGTH = 4;
const int   ORIENTATION_DATA_LENGTH = 3;
const byte  FORWARD_DRIVE = 2;
const byte  NO_DRIVE = 1;
const byte  BACKWARD_DRIVE = 0;
const float DRIVING_SPEED_IN_METER_PER_SECOND = 0.1F;
const byte  LEFT_ROTATION = 2;
const byte  NO_ROTATION = 1;
const byte  RIGHT_ROTATION = 0;
const float ROTATION_SPEED_IN_RAD_PER_SECOND = 0.2618F; //~15 deg/s

//Struct which hold all necessary data for PID controller
struct ControllerData {
  float lastIntegratedError;
  float lastDesiredValue;
  float Kp;
  float Ki;
  float Kd;
};

//union for conversion of 4 bytes to a float value
union uByteFloat {
  byte b[4];
  float fval;
} uBF;

//--------------------------------------------------------------//
//Global variables
//--------------------------------------------------------------//
byte            g_receivedInput[FRAME_LENGTH] = {0};
float           g_receivedPositionData[POSITION_DATA_LENGTH] = {0.F};
float           g_receivedOrientationData[ORIENTATION_DATA_LENGTH] = {0.F};
byte            g_receivedParameterData[FRAME_LENGTH] = {0};
ControllerData  g_positionControllerData;
ControllerData  g_tiltAngleControllerData;
ControllerData  g_yawAngleControllerData;
byte            g_pwmOffsetLeft = 0;
byte            g_pwmOffsetRight = 0;
float           g_pwmLeft = 0.F;
float           g_pwmRight = 0.F;
int             g_pwmLeftOut = 0;
int             g_pwmRightOut = 0;
float           g_pwm = 0.F;
float           g_delta_pwm = 0.F;
float           g_deltaAngleInRadian = 0.F;
float           g_desiredTiltAngle = -25.5F * DEG2RAD;
byte            g_rotateLeftRight = 0;
float           g_lastYawAngle = 0.F;
float           g_givenYawAngleIntegrated = 0.F;
byte            g_driveForwardBackward = 0;
float           g_currentDrivenDistance = 0.F;
float           g_lastDrivenDistance = 0.F;
float           g_desiredDrivenDistance = 0.F;
unsigned long   g_lastTimeStampPosition = (unsigned long)0;
unsigned long   g_lastTimeStampOrientation = (unsigned long)0;
void setup() {
  pinMode(DIRECTION_PIN_LEFT, OUTPUT);
  pinMode(PWM_PIN_LEFT, OUTPUT);
  pinMode(DIRECTION_PIN_RIGHT, OUTPUT);
  pinMode(PWM_PIN_RIGHT, OUTPUT);
  pinMode(LOGIC_VCC_PIN, OUTPUT);
  pinMode(MODE_PIN, OUTPUT);

  digitalWrite(LOGIC_VCC_PIN, HIGH);
  digitalWrite(MODE_PIN, HIGH);

  g_tiltAngleControllerData.lastIntegratedError = 0.F;
  g_tiltAngleControllerData.lastDesiredValue = g_desiredTiltAngle;
  g_tiltAngleControllerData.Kp = 2.65F;
  g_tiltAngleControllerData.Ki = 18.F;
  g_tiltAngleControllerData.Kd = 0.045F;

  g_yawAngleControllerData.lastIntegratedError = 0.F;
  g_yawAngleControllerData.lastDesiredValue = 0.F;
  g_yawAngleControllerData.Kp = 1.0F;
  g_yawAngleControllerData.Ki = .0F;
  g_yawAngleControllerData.Kd = .0F;

  g_positionControllerData.lastIntegratedError = 0.F;
  g_positionControllerData.lastDesiredValue = 0.F;
  g_positionControllerData.Kp = 0.5F;
  g_positionControllerData.Ki = 0.5F;
  g_positionControllerData.Kd = 0.5F;

  Serial.begin(250000);
  while (!Serial);
}

// loop method does the following:
// 1) recive position, orientation data continously and parameter whenever received
//      1.1) position data (z-position and longitudinal velocity) is fed into a PID controller, which outputs a angle difference to the hard coded angle
//           -> This implicitly stabilizes the position of the bot
//      1.2) position data (yaw angle) is fed into a PID controller, which outputs a pwm difference between the left and right motor
//           -> This stabilizes the heading of the bot.
//      1.3) orientation data (tilt angle and velocity) is also fed into a PID controller, which outputs a normalized pwm value
//           -> This stabilizes the angle of the bot (and therefore the position)
//      1.4) use parameter values to set motor offset, PID controller values and driving/rotation direction
// 2) pwm values for the left and right motor and the direction are set as last step
void loop() {

  //Wait for new frame to be fully received
  if (Serial.available() >= FRAME_LENGTH) {
    unsigned long currentTime = micros();
    for (int i = 0; i < FRAME_LENGTH; i++) {
      g_receivedInput[i] = Serial.read();
    }

    //first received byte hold the sensor id
    const byte sensorId = g_receivedInput[0];
    // update global arrays holding latest received values
    updateData(sensorId);

    switch (sensorId) {
      case POSITION_SENSOR:
        {
          //position sensor gives x and z position as well as the longitudnial velcoity and the current yaw angle. This information can also be found within the sensor details inside the app.
          //position data is used to calculate a delta angle for the tilt-angle controller, so the bot holds its position
          const float elapsedTimePos = getElapsedTime(currentTime, g_lastTimeStampPosition);
          updateCurrentDistance(elapsedTimePos);
          updateDesiredDistance(elapsedTimePos);
          const float currentDistanceError = g_desiredDrivenDistance - g_currentDrivenDistance;
          g_deltaAngleInRadian = runPIDController(g_desiredDrivenDistance, currentDistanceError, g_receivedPositionData[2], elapsedTimePos, &g_positionControllerData); // g_receivedPositionData[2] is the current velocity
          //the desired yaw angle is used to get a delta pwm value between the left and right motor. This leads to a rotation of the robot
          const float desiredYawAngle = getDesiredYawAngle(elapsedTimePos);
          const float currentYawAngleError = getSmallestAngleDifference(desiredYawAngle - g_receivedPositionData[3]); // g_receivedPositionData[3] is the current yaw angle
          g_delta_pwm = runPIDController(desiredYawAngle, currentYawAngleError, 0.F, elapsedTimePos, &g_yawAngleControllerData);
          g_lastTimeStampPosition = currentTime;
          //no break - fall through to execute the angle controller as well
        }
      case ORIENTATION_SENSOR:
        {
          //orientation sensor gives tilt angle and velocity. This information can also be found within the sensor details inside the app
          const float elapsedTimeOrientation = getElapsedTime(currentTime, g_lastTimeStampOrientation);
          const float desiredAngle = g_desiredTiltAngle - g_deltaAngleInRadian;
          const float currentTiltAngleError = desiredAngle - g_receivedOrientationData[0];
          g_pwm  = runPIDController(desiredAngle, currentTiltAngleError, g_receivedOrientationData[1], elapsedTimeOrientation, &g_tiltAngleControllerData); // g_receivedOrientationData[1] is the current tilt angle velocity
          g_lastTimeStampOrientation = currentTime;
          break;
        }
      case PARAMETER_SENSOR:
        {
          //parameter "sensor" used here to parameterize the 3 PID controllers and other stuff
          //the parameter sensor sends 12bytes which can be interpreted as you wish
          g_pwmOffsetLeft   = g_receivedParameterData[1];
          g_pwmOffsetRight  = g_receivedParameterData[2];
          g_desiredTiltAngle =  -(float)g_receivedParameterData[3] * TILT_ANGLE_QUANTIZATION; // note the minus here
          g_tiltAngleControllerData.Kp =  (float)g_receivedParameterData[4] * TILT_CONTROLLER_P_QUANTIZATION;
          g_tiltAngleControllerData.Ki =  (float)g_receivedParameterData[5] * TILT_CONTROLLER_I_QUANTIZATION;
          g_tiltAngleControllerData.Kd =  (float)g_receivedParameterData[6] * TILT_CONTROLLER_D_QUANTIZATION;
          g_positionControllerData.Kp =   (float)g_receivedParameterData[7] * POS_CONTROLLER_QUANTIZATION;
          g_positionControllerData.Ki =   (float)g_receivedParameterData[8] * POS_CONTROLLER_QUANTIZATION;
          g_positionControllerData.Kd =   (float)g_receivedParameterData[9] * POS_CONTROLLER_QUANTIZATION;
          g_yawAngleControllerData.Kp =   (float)g_receivedParameterData[10] * YAW_CONTROLLER_QUANTIZATION;
          g_driveForwardBackward  = g_receivedParameterData[11];
          g_rotateLeftRight       = g_receivedParameterData[12];
          break;
        }
      default:
        {
          //do nothing
          break;
        }
    }

    const float pwmLeft = getAbsPwm(g_pwm - g_delta_pwm);
    const float pwmRight = getAbsPwm(g_pwm + g_delta_pwm);

    g_pwmLeftOut = getPWMValueForOutput(pwmLeft, g_pwmOffsetLeft);
    g_pwmRightOut = getPWMValueForOutput(pwmRight, g_pwmOffsetRight);

    analogWrite(PWM_PIN_LEFT, g_pwmLeftOut);
    analogWrite(PWM_PIN_RIGHT, g_pwmRightOut);
  }
}

// getAbsPwm method - get abs value of current signed pwm value and set direction pins accordingly
// Input:
// * pwm - current pwm controller value
// Returns: pwm as float in range from 0...1
float getAbsPwm(float pwm) {
  if (pwm < 0.F) {
    digitalWrite(DIRECTION_PIN_LEFT, HIGH);
    digitalWrite(DIRECTION_PIN_RIGHT, HIGH);
    pwm *= -1.F;
  } else {
    digitalWrite(DIRECTION_PIN_LEFT, LOW);
    digitalWrite(DIRECTION_PIN_RIGHT, LOW);
  }
  return min(pwm, 1.F);
}

// getPWMValueForOutput method - map normed pwm to integer output
// Input:
// * pwmNormed - normed pwm (0...1), will be mapped to 0...MAX_PWM
// * offset - gets added to the above result (motor friction)
// Returns: int in range from 0...MAX_PWM
int getPWMValueForOutput(float pwmNormed, byte offset) {
  return min((int)((pwmNormed) * (float)MAX_PWM) + (int)offset, MAX_PWM);
}

// updateData method - writes the received sensor data to the respective global data array
// Input:
// * sensorId - id of the received sensor data
// Returns: nothing
void updateData(const byte sensorId) {
  switch (sensorId) {
    case POSITION_SENSOR:
      {
        for (int i = 0; i < POSITION_DATA_LENGTH; i++) {
          byte offset = i * BYTES_PER_POSITION_VALUE;
          g_receivedPositionData[i] = getFloatFrom3Bytes(g_receivedInput, offset);
        }
        break;
      }
    case ORIENTATION_SENSOR:
      {
        for (int i = 0; i < ORIENTATION_DATA_LENGTH; i++) {
          uBF.b[0] = g_receivedInput[1 + i * BYTES_PER_FLOAT];
          uBF.b[1] = g_receivedInput[2 + i * BYTES_PER_FLOAT];
          uBF.b[2] = g_receivedInput[3 + i * BYTES_PER_FLOAT];
          uBF.b[3] = g_receivedInput[4 + i * BYTES_PER_FLOAT];
          g_receivedOrientationData[i] = uBF.fval;
        }
        break;
      }
    case PARAMETER_SENSOR:
      {
        //parameter sensor sends bytes with user defined meaning, so just copy them
        for (int i = 0; i < FRAME_LENGTH; i++) {
          g_receivedParameterData[i] = g_receivedInput[i];
        }
        break;
      }
    default:
      {
      }
  }
}

// getFloatFrom3Bytes method - gets a float value from a 3byte fixed point vlaue
// Input:
// * receivedBytes - received byte array
// * offset - current offset, multiples of 3 (since every value has 24byte)
// Returns: received float value at offset position
float getFloatFrom3Bytes(const byte receivedBytes[], const byte offset) {
  long intRepresentation = (
                             ((0xFFL & receivedBytes[3 + offset]) << 16) |
                             ((0xFFL & receivedBytes[2 + offset]) << 8) |
                             (0xFFL & receivedBytes[1 + offset])
                           );

  if ((intRepresentation & 0x00800000) > 0) {
    intRepresentation |= 0xFF000000;
  } else {
    intRepresentation &= 0x00FFFFFF;
  }
  //65536.F is the norming factor of the fixed point value
  return intRepresentation / 65536.F;
}

// runPIDController - executes a PID controller. This implementation takes the derivate directly into account which improves performance in case you can measure it directly (avoids calculating the numerical derivative)
// Input:
// * desiredValue - value you want to achieve
// * error - current error defined as: desiredValue - currentValue
// * currentValueDerivative - derivative of the current value (what you have measured)
// * elapsedTime - elapsed time since last call in seconds
// * controllerData - holds P,I and D parameters and other values from previous execution
// Returns: new controller output
float runPIDController(const float desiredValue, const float error, const float currentValueDerivative, const float elapsedTime, ControllerData* controllerData ) {
  float output = 0.F;
  if (elapsedTime > 0) {
    const float integratedError = controllerData->lastIntegratedError + error * elapsedTime;                             //numerical integration of the error
    const float rateError = ((desiredValue - controllerData->lastDesiredValue) / elapsedTime) - currentValueDerivative;  //rate error for given derivative, avoids calculating the numerical derivative again (at least for the measured part)

    output = controllerData->Kp * error + controllerData->Ki * integratedError + controllerData->Kd * rateError;
    controllerData->lastIntegratedError = integratedError;
  }
  //values for the next call
  controllerData->lastDesiredValue = desiredValue;
  return output;
}

// getSmallestAngleDifference - get the smallest angle difference
// Input:
// * angleDiff - angle difference between two angles, angles which are substracted must be in the range of -Pi to Pi!
// Returns: smallest angle difference
float getSmallestAngleDifference(const float angleDiff) {
  if (angleDiff > (float)PI) {
    return angleDiff - 2.F * PI;
  }
  if (angleDiff < -(float)PI) {
    return angleDiff + 2.F * PI;
  }
  return angleDiff;
}

bool isDrivingOrRotating() {
  return  (g_driveForwardBackward == FORWARD_DRIVE) || (g_driveForwardBackward == BACKWARD_DRIVE) || (g_rotateLeftRight == LEFT_ROTATION) || (g_rotateLeftRight == RIGHT_ROTATION);
}

// updateCurrentDistance - update current g_currentDrivenDistance depending on current driving input
// Input:
// * deltaTime - time between two successive calls
void updateCurrentDistance(float deltaTime) {
  if (isDrivingOrRotating()) {
    //integrate velocity to reflect position change
    g_currentDrivenDistance += g_receivedPositionData[2] * deltaTime;
  } else {
    //set pos-z as current value
    g_currentDrivenDistance = g_receivedPositionData[1];
  }
}
// updateDesiredDistance - update the desired driven distance for the distance controller. While there is any movement input received, the z position is stored and set as desired when movement is over.
// While driving, a defined speed is used to increment the desired position accordingly
// Input:
// * deltaTime - time between two successive calls
void updateDesiredDistance(float deltaTime) {
  if (isDrivingOrRotating()) {
    g_lastDrivenDistance = g_receivedPositionData[1];
  } else {
    g_desiredDrivenDistance = g_lastDrivenDistance;
    g_positionControllerData.lastDesiredValue = g_desiredDrivenDistance; // not so nice to do it here, but avoids having big derivatives in the position pid controller
  }
  switch (g_driveForwardBackward) {
    case FORWARD_DRIVE:
      g_desiredDrivenDistance += deltaTime * DRIVING_SPEED_IN_METER_PER_SECOND;
      break;
    case BACKWARD_DRIVE:
      g_desiredDrivenDistance -= deltaTime * DRIVING_SPEED_IN_METER_PER_SECOND;
      break;
    default:
      //do nothing
      break;
  }
}

// getDesiredYawAngle - get the desired yaw angle for the yaw angle (heading-) controller. Checks for the received rotation direction and changes the desired heading accordingly
// Input:
// * deltaTime - time between two successive calls
// Returns: desired heading
float getDesiredYawAngle(float deltaTime) {
  switch (g_rotateLeftRight) {
    case LEFT_ROTATION:
      g_lastYawAngle = g_receivedPositionData[3];
      g_givenYawAngleIntegrated += deltaTime * ROTATION_SPEED_IN_RAD_PER_SECOND;
      g_givenYawAngleIntegrated = getSmallestAngleDifference(g_givenYawAngleIntegrated);
      return g_givenYawAngleIntegrated;
    case RIGHT_ROTATION:
      g_lastYawAngle = g_receivedPositionData[3];
      g_givenYawAngleIntegrated -= deltaTime * ROTATION_SPEED_IN_RAD_PER_SECOND;
      g_givenYawAngleIntegrated = getSmallestAngleDifference(g_givenYawAngleIntegrated);
      return g_givenYawAngleIntegrated;
    default:
      g_givenYawAngleIntegrated = g_lastYawAngle;
      return g_lastYawAngle;
  }
}

// getElapsedTime - get elapsed time between to timestamps
// Input:
// * currentTime  - current time stamp
// * previousTime - previous time stamp
// Returns: elapesd time in seconds
float getElapsedTime(const unsigned long currentTime, const unsigned long previousTime) {
  if (previousTime == 0) {
    return 0.F;
  } else {
    return (currentTime - previousTime) * MICRO2SECONDS;
  }
}
