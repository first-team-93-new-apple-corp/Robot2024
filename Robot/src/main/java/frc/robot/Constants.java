package frc.robot;

public final class Constants {

  public class Joystick_Port {
    public static final int Driver1Port = 0;
    public static final int Driver2Port = 1;
    public static final int Operator1Port = 2;
    public static final int Operator2Port = 3;

  }


  public class OperatorSettings{
    public static final double TimeBetweenSelectorPresses = 0.25;
  }

  public class CanID_Rev {
    public static final int PDH = 1; 
    public static final int WristMotor = 4; 

    public static final int GrabberMotor1 = 2; 
    public static final int GrabberMotor2 = 5; 
    

  }

  public class CanID_CTRE {

    // Drive Motors
    public static final int FrontLeftDrive = 1;
    public static final int FrontRightDrive = 2;
    public static final int BackRightDrive = 3;
    public static final int BackLeftDrive = 4;

    // Drive Turning Motors
    public static final int FrontLeftTurn = 5;
    public static final int FrontRightTurn = 6;
    public static final int BackRightTurn = 7;
    public static final int BackLeftTurn = 8;

    // Drive Encoders
    public static final int FrontLeftCancoder = 10;
    public static final int FrontRightCancoder = 11;
    public static final int BackRightCancoder = 12;
    public static final int BackLeftCancoder = 13;

    // Shoulder
    public static final int FrontLeftShoulder = 14;
    public static final int FrontRightShoulder = 15;
    public static final int BackRightShoulder = 16;
    public static final int BackLeftShoulder = 17;

    public static final int ShoulderCancoder = 19; 

    // Telescoping
    public static final int TelescopingMotor = 18; 

    // Wrist 
    // CONFIRM
    public static final int WristCancoder = 20;


    // 21

  }

  public class F310 {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LeftShoulderButton = 5;
    public static final int RightShoulderButton = 6;
    public static final int Back = 7;
    public static final int Start = 8;
    public static final int LeftStick = 9;
    public static final int RightStick = 10;
  }

  public class LogitechX3D {
    // BUTTON ASSIGNMENT FOR LOGITECH EXTREME 3D PRO
    public static final int Trigger = 1;
    public static final int Thumb = 2;
    public static final int StickLowerLeft = 3;
    public static final int StickLowerRight = 4;
    public static final int StickUpperRight = 5;
    public static final int StickUpperLeft = 6;
    public static final int BaseForwardLeft = 7;
    public static final int BaseForwardRight = 8;
    public static final int BaseMiddleLeft = 9;
    public static final int BaseMiddleRight = 10;
    public static final int BaseBackLeft = 11;
    public static final int BaseBackRight = 12;
  }

  public class Shoulder {

  }

  public class Telescoping {
    public static final int ExtendedLimitSwitchPort = 0; 
    public static final int RetractedLimitSwitchPort = 1;
  }

  public class Wrist{
    public static final int GearRatio = 120;
    public static final double configMagnetOffset = -47.5;
    public static final double kP = 0.0004;
    public static final double MaxAccel = 5000;
    public static final double MaxVelo = 5000;
    public static final double MaxError = 0.1;

  }


  // TODO: Organize these
  public static final double DegreesToRotationsShoulder = 0.0;
  public static final double InchesToRotationsTelescope = 0.0;
  public static final double MaxAngle = 0;
  public static final double MinAngle = 0;
  public static final double totalTicks = 13943;
  public static final double MAXActuation = 16;
  public static final double InchesToTicksTelescope = totalTicks / MAXActuation;
  public static final double ShoulderGearRatio = 240; // TODO GET ACTUAL VALUE

  // 2048 ticks per revolution * 120 gear ratio * 2:1 ratio between motor and arm angle
  public static final double DegreesToTicksShoulder =  (2048 * 120 * 2);


  public static final double checkJoystickDeadband(double joystickValue, double deadband) {
    if (Math.abs(joystickValue) < deadband) {
      joystickValue = 0;
    }

    return joystickValue;
  }

}
