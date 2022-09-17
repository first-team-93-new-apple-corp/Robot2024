package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final double TurningTolerance = 0.1;
    public static final double TurningP = 0;
    public static final double TurningI = 0;
    public static final double TurningD = 0;

    public static final double DrivingTolerance = 0.1;
    public static final double ThrottleP = 0;
    public static final double ThrottleI = 0;
    public static final double ThrottleD = 0;

    public static final int FLThrottlePort = 1;
    public static final int FRThrottlePort = 2;
    public static final int BLThrottlePort = 3;
    public static final int BRThrottlePort = 4;

    public static final int FLTurningPort = 5;
    public static final int FRTurningPort = 6;
    public static final int BLTurningPort = 7;
    public static final int BRTurningPort = 8;

    public static final int FLEncoderPort = 9;
    public static final int FREncoderPort = 10;
    public static final int BLEncoderPort = 11;
    public static final int BREncoderPort = 12;

    public static final double FLMagnetOffset = 0.0;
    public static final double FRMagnetOffset = 0.0;
    public static final double BLMagnetOffset = 0.0;
    public static final double BRMagnetOffset = 0.0;


    // Motor Information
    public static final double TalonFXRPM = 6380;
    public static final double TalonFXRPS = TalonFXRPM / 60;
    public static final double TalonFXEncoderResolution = 2048;

    // wheel Information
    public static final double WheelRadius = 2;
    public static final double WheelCircumference = WheelRadius * 2 * Math.PI;

    // Drivetrain Information
    public static final double TrackWidth = Units.inchesToMeters(26); // right wheel to left wheel
    public static final double WheelBase = Units.inchesToMeters(26); //Front To Back
    public static final double TurningGearing = 12.8;
    public static final double drivingGearing = 6.75;
    public static final double maxStrafeSpeed = TalonFXRPS / drivingGearing * WheelCircumference; // m/s
    public static final double maxAngularSpeed = 2; // m/s
    public static final double maxAngularAcceleration = 2 * Math.PI;

    //SwerveLocations
    public static final Translation2d locationFL = new Translation2d(WheelBase / 2, TrackWidth / 2);
    public static final Translation2d locationFR = new Translation2d(WheelBase / 2, -TrackWidth / 2);
    public static final Translation2d locationBL = new Translation2d(-WheelBase / 2, TrackWidth / 2);
    public static final Translation2d locationBR = new Translation2d(-WheelBase / 2, -TrackWidth / 2);

    public static final double MaxVolts = 12.0;

}