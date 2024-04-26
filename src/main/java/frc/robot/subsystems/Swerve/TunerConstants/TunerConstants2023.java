package frc.robot.subsystems.Swerve.TunerConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class TunerConstants2023 {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
                        .withKP(100).withKI(0).withKD(0.2)
                        .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
                        .withKP(3).withKI(0).withKD(0)
                        .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 4.73;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.746031746031747;
        private static final double kSteerGearRatio = 12.8;
        private static final double kWheelRadiusInches = 2;

        private static final boolean kSteerMotorReversed = false;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "Platform";
        private static final int kPigeonId = 0;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        private static SwerveDrivetrainConstants DrivetrainConstants;

        private static SwerveModuleConstantsFactory ConstantCreator;
        // Front Left
        private static final int kFrontLeftDriveMotorId = 1;
        private static final int kFrontLeftSteerMotorId = 5;
        private static final int kFrontLeftEncoderId = 10;
        private static final double kFrontLeftEncoderOffset = 0.1181640625;

        private static final double kFrontLeftXPosInches = 9.25;
        private static final double kFrontLeftYPosInches = 9.25;

        // Front Right
        private static final int kFrontRightDriveMotorId = 2;
        private static final int kFrontRightSteerMotorId = 6;
        private static final int kFrontRightEncoderId = 11;
        private static final double kFrontRightEncoderOffset = -0.081787109375;

        private static final double kFrontRightXPosInches = 9.25;
        private static final double kFrontRightYPosInches = -9.25;

        // Back Left
        private static final int kBackLeftDriveMotorId = 4;
        private static final int kBackLeftSteerMotorId = 8;
        private static final int kBackLeftEncoderId = 13;
        private static final double kBackLeftEncoderOffset = 0.233642578125;

        private static final double kBackLeftXPosInches = -9.25;
        private static final double kBackLeftYPosInches = 9.25;

        // Back Right
        private static final int kBackRightDriveMotorId = 3;
        private static final int kBackRightSteerMotorId = 7;
        private static final int kBackRightEncoderId = 12;
        private static final double kBackRightEncoderOffset = 0.3466796875;

        private static final double kBackRightXPosInches = -9.25;
        private static final double kBackRightYPosInches = -9.25;

        private static SwerveModuleConstants FrontLeft;
        private static SwerveModuleConstants FrontRight;
        private static SwerveModuleConstants BackLeft;
        private static SwerveModuleConstants BackRight;

        public SwerveDriveSubsystem DriveTrain;

        public TunerConstants2023() {
                DrivetrainConstants = new SwerveDrivetrainConstants()
                                .withPigeon2Id(kPigeonId)
                                .withCANbusName(kCANbusName);

                ConstantCreator = new SwerveModuleConstantsFactory()
                                .withDriveMotorGearRatio(kDriveGearRatio)
                                .withSteerMotorGearRatio(kSteerGearRatio)
                                .withWheelRadius(kWheelRadiusInches)
                                .withSlipCurrent(kSlipCurrentA)
                                .withSteerMotorGains(steerGains)
                                .withDriveMotorGains(driveGains)
                                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                                .withSteerInertia(kSteerInertia)
                                .withDriveInertia(kDriveInertia)
                                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                                .withCouplingGearRatio(kCoupleRatio)
                                .withSteerMotorInverted(kSteerMotorReversed);

                FrontLeft = ConstantCreator.createModuleConstants(
                                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
                                kFrontLeftEncoderOffset,
                                Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
                                kInvertLeftSide);
                FrontRight = ConstantCreator.createModuleConstants(
                                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                                kFrontRightEncoderOffset,
                                Units.inchesToMeters(kFrontRightXPosInches),
                                Units.inchesToMeters(kFrontRightYPosInches),
                                kInvertRightSide);
                BackLeft = ConstantCreator.createModuleConstants(
                                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
                                kBackLeftEncoderOffset,
                                Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
                                kInvertLeftSide);
                BackRight = ConstantCreator.createModuleConstants(
                                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
                                kBackRightEncoderOffset,
                                Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
                                kInvertRightSide);

                DriveTrain = new SwerveDriveSubsystem(DrivetrainConstants, FrontLeft,
                                FrontRight, BackLeft, BackRight);
        }
}