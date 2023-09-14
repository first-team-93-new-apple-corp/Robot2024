package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public final class Constants {

  public class Joystick_Port {

    public static final int Driver1Port = 0;
    public static final int Driver2Port = 1;
    public static final int Operator1Port = 2;
    public static final int Operator2Port = 3;
  }

  public class OperatorSettings {

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

    // 2048 ticks per revolution * 120 gear ratio * 2:1 ratio between motor and arm angle
    public static final double DegreesToTicksShoulder = (2048 * 120 * 2);
  }

  public class Telescoping {

    public static final int ExtendedLimitSwitchPort = 0;
    public static final int RetractedLimitSwitchPort = 1;
  }

  public class Wrist {

    public static final int GearRatio = 120;
    public static final double configMagnetOffset = -47.5;
    public static final double kP = 0.0004;
    public static final double MaxAccel = 5000;
    public static final double MaxVelo = 5000;
    public static final double MaxError = 0.1;
  }

  public static class Drive {

    // Turning PID
    public static final double Turning_Tolerance = SwerveModule.degreesToTicks(
      0.75
    );
    public static double Turning_P = 0.3;
    public static double Turning_I = 0;
    public static double Turning_D = 0; // 0.05;

    // Front Left Module
    public static final int Throttle_Port_FL = 1;
    public static final int Turning_Port_FL = 5;
    public static final int Encoder_Port_FL = 10;
    public static final double Magnet_Offset_FL = 41.836;

    // Front Right Module
    public static final int Throttle_Port_FR = 2;
    public static final int Turning_Port_FR = 6;
    public static final int Encoder_Port_FR = 11;
    public static final double Magnet_Offset_FR = 151.347;

    // Back Right Module
    public static final int Throttle_Port_BR = 3;
    public static final int Turning_Port_BR = 7;
    public static final int Encoder_Port_BR = 12;
    public static final double Magnet_Offset_BR = 124.541;

    // Back Left Module
    public static final int Throttle_Port_BL = 4;
    public static final int Turning_Port_BL = 8;
    public static final int Encoder_Port_BL = 13;
    public static final double Magnet_Offset_BL = -94.483;

    // Motor Information
    public static final double TalonFX_RPM = 6380;
    public static final double TalonFX_RPS = TalonFX_RPM / 60;
    public static final double TalonFX_Encoder_Resolution = 2048;

    // Wheel Information
    public static final double Wheel_Radius = Units.inchesToMeters(4 / 2);
    public static final double Wheel_Circumference = Wheel_Radius * 2 * Math.PI;

    // Drivetrain Information
    // public static final double Track_Width = Units.inchesToMeters(29); // right wheel to left wheel
    // public static final double Wheel_Base = Units.inchesToMeters(29); //Front To Back
    public static final double Track_Width = Units.inchesToMeters(25); //23 bot
    public static final double Wheel_Base = Units.inchesToMeters(25); //23 bot

    // Gearing Ratios
    public static final double Turning_Gearing = 12.8;
    // public static final double Driving_Gearing = 6.12;
    public static final double Driving_Gearing = 6.75;

    // Speed Limiters
    public static final double Max_Strafe_Speed =
      (TalonFX_RPS / Driving_Gearing) * Wheel_Circumference; // m/s
    public static final double Max_Angular_Speed = 2 * Math.PI; // m/s
    public static final double Max_Angular_Acceleration = 2 * Math.PI;
    public static final double Max_Acceleration = 2;
    public static final double Max_Volts = 12.0;

    //Swerve Locations
    // public static Translation2d Center = new Translation2d(0, 0);
    public static final Translation2d Center = new Translation2d(0, 0);

    public static final Translation2d Front = new Translation2d(
      Wheel_Base / 2,
      0
    );
    public static final Translation2d Right = new Translation2d(
      0,
      -Track_Width / 2
    );
    public static final Translation2d Back = new Translation2d(
      -Wheel_Base / 2,
      0
    );
    public static final Translation2d Left = new Translation2d(
      0,
      Track_Width / 2
    );

    public static final Translation2d Location_FL = new Translation2d(
      Wheel_Base / 2,
      Track_Width / 2
    );
    public static final Translation2d Location_FR = new Translation2d(
      Wheel_Base / 2,
      -Track_Width / 2
    );
    public static final Translation2d Location_BL = new Translation2d(
      -Wheel_Base / 2,
      Track_Width / 2
    );
    public static final Translation2d Location_BR = new Translation2d(
      -Wheel_Base / 2,
      -Track_Width / 2
    );
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

  public static final double checkJoystickDeadzone(
    double joystickValue,
    double deadzone
  ) {
    if (Math.abs(joystickValue) < deadzone) {
      joystickValue = 0;
    }
    return joystickValue;
  }
}
