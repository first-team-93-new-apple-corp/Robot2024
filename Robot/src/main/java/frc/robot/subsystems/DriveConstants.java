package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import javax.sound.midi.Track;

public final class DriveConstants {
  public static final double Turning_Tolerance = Units.degreesToRadians(1);
  public static double Turning_P = 0.3;
  public static double Turning_I = 0;
  public static double Turning_D = 0;
  public static double Turning_F = 0.0;

  public static final double Throttle_Tolerance = 0.1;

  public static final int Throttle_Port_FL = 1;
  public static final int Turning_Port_FL = 5;
  public static final int Encoder_Port_FL = 10;
  public static final double Magnet_Offset_FL = -136.318 + 180 - 1.846;/*TODO Check FL offset for error because of the seen 90 degree error */

  public static final int Throttle_Port_BR = 3;
  public static final int Turning_Port_BR = 7;
  public static final int Encoder_Port_BR = 12;
  public static final double Magnet_Offset_BR = 124.541;

  public static final int Throttle_Port_BL = 4;
  public static final int Turning_Port_BL = 8;
  public static final int Encoder_Port_BL = 13;
  public static final double Magnet_Offset_BL = 85.078 - 180 + 0.439;

  public static final int Throttle_Port_FR = 2;
  public static final int Turning_Port_FR = 6;
  public static final int Encoder_Port_FR = 11;
  public static final double Magnet_Offset_FR = 149.941 + 1.406;

  // Motor Information
  public static final double TalonFX_RPM = 6380;
  public static final double TalonFX_RPS = TalonFX_RPM / 60;
  public static final double TalonFX_Encoder_Resolution = 2048;

  // wheel Information
  public static final double Wheel_Radius = 0.0508;
  public static final double Wheel_Circumference = Wheel_Radius * 2 * Math.PI;

  // Drivetrain Information
  public static final double Track_Width = Units.inchesToMeters(29); // right wheel to left wheel
  public static final double Wheel_Base = Units.inchesToMeters(29); //Front To Back

  //TODO: UPDATE
  public static final double Turning_Gearing = 12.8;
  public static final double Driving_Gearing = 6.75;

  public static final double Max_Strafe_Speed =
    TalonFX_RPS / Driving_Gearing * Wheel_Circumference; // m/s
  public static final double Max_Angular_Speed = 2 * Math.PI; // m/s
  public static final double Max_Angular_Acceleration = 2 * Math.PI;

  //Swerve Locations
  public static final Translation2d Center = new Translation2d(-.15, .05); /*TODO For Nolen: Reminder to look into this tomorrow, may be causing issues*/
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

  public static final double Max_Volts = 12.0;
}
