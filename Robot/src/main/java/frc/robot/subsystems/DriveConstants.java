package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final double Turning_Tolerance = 0.1;
    public static final double Turning_P = 0;
    public static final double Turning_I = 0;
    public static final double Turning_D = 0;

    public static final double Throttle_Tolerance = 0.1;
    public static final double Throttle_P = 0;
    public static final double Throttle_I = 0;
    public static final double Throttle_D = 0;


    //TODO: Setup ports before testing
    public static final int Throttle_Port_FL = 0;
    public static final int Throttle_Port_FR = 0;
    public static final int Throttle_Port_BL = 0;
    public static final int Throttle_Port_BR = 0;

    public static final int Turning_Port_FL = 0;
    public static final int Turning_Port_FR = 0;
    public static final int Turning_Port_BL = 0;
    public static final int Turning_Port_BR = 0;

    public static final int Encoder_Port_FL = 0;
    public static final int Encoder_Port_FR = 0;
    public static final int Encoder_Port_BL = 0;
    public static final int Encoder_Port_BR = 0;

    public static final double Magnet_Offset_FL = 0.0;
    public static final double Magnet_Offset_FR = 0.0;
    public static final double Magnet_Offset_BL = 0.0;
    public static final double Magnet_Offset_BR = 0.0;


    // Motor Information
    public static final double TalonFX_RPM = 6380;
    public static final double TalonFX_RPS = TalonFX_RPM / 60;
    public static final double TalonFX_Encoder_Resolution = 2048;

    // wheel Information
    public static final double Wheel_Radius = 2;
    public static final double Wheel_Circumference = Wheel_Radius * 2 * Math.PI;

    // Drivetrain Information
    public static final double Trackwidth = Units.inchesToMeters(26); // right wheel to left wheel
    public static final double Wheel_Base = Units.inchesToMeters(26); //Front To Back
    public static final double Turning_Gearing = 12.8;
    public static final double Driving_Gearing = 6.75;
    public static final double Max_Strafe_Speed = TalonFX_RPS / Driving_Gearing * Wheel_Circumference; // m/s
    public static final double Max_Angular_Speed = 2; // m/s
    public static final double Max_Angular_Acceleration = 2 * Math.PI;

    //Swerve Locations
    public static final Translation2d Location_FL = new Translation2d(Wheel_Base / 2, Trackwidth / 2);
    public static final Translation2d Location_FR = new Translation2d(Wheel_Base / 2, -Trackwidth / 2);
    public static final Translation2d Location_BL = new Translation2d(-Wheel_Base / 2, Trackwidth / 2);
    public static final Translation2d Location_BR = new Translation2d(-Wheel_Base / 2, -Trackwidth / 2);

    public static final double Max_Volts = 12.0;

}