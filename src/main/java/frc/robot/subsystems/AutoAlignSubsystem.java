package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutoAlignSubsystem extends SubsystemBase {
    public final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;
    // private SwerveRequest.ApplyChassisSpeeds m_swerveRequest = new
    // SwerveRequest.ApplyChassisSpeeds();
    // private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; //
    // My drivetrain
    ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    // ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    SwerveDriveSubsystem drivetrain;
    ChassisSpeeds alignSpeeds; // Chassis Speeds which robot uses for auto align

    ChassisSpeeds fieldSpeeds;
    PIDController AlignPIDTheta = new PIDController(.15, 0, 0.01); // Rotationly PID
    PIDController AlignPIDY = new PIDController(1, 0, 0);
    PIDController AlignPIDX = new PIDController(1, 0, 0);
    ProfiledPIDController AlignPIDTheta2 = new ProfiledPIDController(.15, 0, 0.01, new TrapezoidProfile.Constraints(DriveConstants.MaxAngularRate, 12));

    double X, Y, Theta;

    double AmpSetpointX = 1.75, AmpSetpointY = 7.6, AmpSetpointTheta = -90;
    double TrapSetpoint1X = 4.1, TrapSetpoint1Y = 2.8, TrapSetpoint1Theta = -120;
    double TrapSetpoint2X = 4, TrapSetpoint2Y = 5.2, TrapSetpoint2Theta = 120;
    double TrapSetpoint3X = 6.2, TrapSetpoint3Y = 4, TrapSetpoint3Theta = 0;

    double toleranceX = .04, toleranceY = .04, toleranceTheta = Math.toRadians(3);

    boolean red;

    // Optional<Alliance> ally = DriverStation.getAlliance();
    public AutoAlignSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        AlignPIDTheta.enableContinuousInput(-Math.PI, Math.PI);
        AlignPIDTheta2.enableContinuousInput(-Math.PI, Math.PI);
        AlignPIDTheta.setIntegratorRange(-0.3, 0.3);
        AlignPIDTheta2.setIntegratorRange(-0.3, 0.3);
        AlignPIDX.setIntegratorRange(-0.3, 0.3);
        AlignPIDY.setIntegratorRange(-0.3, 0.3);
        if (true) {
            if (true) {
                AmpSetpointX = 14.6;
                AmpSetpointY = 7.6;
                AmpSetpointTheta = Math.toRadians(-90);

                TrapSetpoint1X = 12.5;
                TrapSetpoint1Y = 2.8;
                TrapSetpoint1Theta = Math.toRadians(-60);

                TrapSetpoint2X = 12.258;
                TrapSetpoint2Y = 5.038;
                TrapSetpoint2Theta = Math.toRadians(60);

                TrapSetpoint3X = 10.4;
                TrapSetpoint3Y = 4;
                TrapSetpoint3Theta = Math.toRadians(180);
            } else if (true) {
                AmpSetpointX = 1.75;
                AmpSetpointY = 7.6;
                AmpSetpointTheta = -90;

                TrapSetpoint1X = 4.1;
                TrapSetpoint1Y = 2.8;
                TrapSetpoint1Theta = -120;

                TrapSetpoint2X = 4;
                TrapSetpoint2Y = 5.2;
                TrapSetpoint2Theta = 120;

                TrapSetpoint3X = 6.2;
                TrapSetpoint3Y = 4;
                TrapSetpoint3Theta = 180;
            }
        }
    }

    public void Alliance() {
        // if (SmartDashboard.getBoolean("Red?", red)) {
        // AmpSetpointX = 14.6;
        // AmpSetpointY = 7.6;
        // AmpSetpointTheta = -90;

        // TrapSetpoint1X = 12.5;
        // TrapSetpoint1Y = 2.8;
        // TrapSetpoint1Theta = -30;

        // TrapSetpoint2X = 12.6;
        // TrapSetpoint2Y = 5.2;
        // TrapSetpoint2Theta = 30;

        // TrapSetpoint3X = 10.4;
        // TrapSetpoint3Y = 4;
        // TrapSetpoint3Theta = 0;
        // } else {
        // AmpSetpointX = 1.75;
        // AmpSetpointY = 7.6;
        // AmpSetpointTheta = -90;

        // TrapSetpoint1X = 4.1;
        // TrapSetpoint1Y = 2.8;
        // TrapSetpoint1Theta = -60;

        // TrapSetpoint2X = 4;
        // TrapSetpoint2Y = 5.2;
        // TrapSetpoint2Theta = 60;

        // TrapSetpoint3X = 6.2;
        // TrapSetpoint3Y = 4;
        // TrapSetpoint3Theta = 180;
        // }
    }

    public void updateValues(double PIDSetpointX, double PIDSetpointY, double PIDSetpointTheta) {
        drivetrain.updateOdometry();
        X = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getX();
        Y = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getY();
        Theta = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians();

        alignSpeeds = new ChassisSpeeds(
                ((MathUtil.clamp((AlignPIDX.calculate(X, PIDSetpointX)), -MaxSpeed, MaxSpeed)) * -1), // Velocity X
                                                                                                      // Sawyer
                ((MathUtil.clamp((AlignPIDY.calculate(Y, PIDSetpointY)), -MaxSpeed, MaxSpeed)) * -1), // Velocity Y
                                                                                                      // Sawyer
                (-MathUtil.clamp((AlignPIDTheta2.calculate(Theta, (PIDSetpointTheta))), -MaxAngularRate, MaxAngularRate))); // Rotational
                                                                                                                         // Speeds
        // (-MathUtil.clamp((AlignPIDTheta.calculate(Math.toRadians(Theta),
        // Math.toRadians(PIDSetpointTheta))), -MaxAngularRate, MaxAngularRate))); //
        // Rotational Speeds

        fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(alignSpeeds,
                new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
                        .rotateBy(new Rotation2d(0)));

    }

    public void AutoAimAmp() { // Works amp
        AutoAim(AmpSetpointX, AmpSetpointY, AmpSetpointTheta);
    }

    public void AutoAimTrap() { // Works with all trap locations
        if (true) {
            // red
            if (Y < 4 && X > 10.75) {
                // Trap 1
                AutoAim(TrapSetpoint1X, TrapSetpoint1Y, TrapSetpoint1Theta);
            } else if (Y > 4 && X > 10.75) {
                // Trap 2
                AutoAim(TrapSetpoint2X, TrapSetpoint2Y, TrapSetpoint2Theta);
            } else if (X < 10.75) {
                // Trap 3
                AutoAim(TrapSetpoint3X, TrapSetpoint3Y, TrapSetpoint3Theta);
            }
        } else {
            // blue
            if (Y < 4 && X < 5.3) {
                // Trap 1
                AutoAim(TrapSetpoint1X, TrapSetpoint1Y, TrapSetpoint1Theta);
            } else if (Y > 4 && X < 5.3) {
                // Trap 2
                AutoAim(TrapSetpoint2X, TrapSetpoint2Y, TrapSetpoint2Theta);
            } else if (X > 5.3) {
                // Trap 3
                AutoAim(TrapSetpoint3X, TrapSetpoint3Y, TrapSetpoint3Theta);
            } else {
            }
        }
    }

    public void AutoAim(double SetpointX, double SetpointY, double SetpointTheta) {
        updateValues(SetpointX, SetpointY, SetpointTheta);

        if (!((Math.abs(X - SetpointX) < toleranceX)
                &
                ((Math.abs(Y - SetpointY) < toleranceY)
                &
                (Math.abs((-1*Math.PI) + Math.abs(Theta) + Math.abs(SetpointTheta)) < toleranceTheta)))) {
            
                    drivetrain.driveRobotRelative(fieldSpeeds);

        } else {
        }

    }

    @Override
    public void periodic() {
    }
}
