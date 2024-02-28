package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class AutoAlignSubsystem extends SubsystemBase {
    public final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;

    ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    // ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    SwerveDriveSubsystem drivetrain;
    ChassisSpeeds alignSpeeds; // Chassis Speeds which robot uses for auto align

    ChassisSpeeds fieldSpeeds;
    PIDController AlignPIDTheta = new PIDController(.19, 0, 0.15); // Rotationly PID
    PIDController AlignPIDX = new PIDController(2.1, 0.14, 0.2); //Drive PIDs should be the same
    PIDController AlignPIDY = new PIDController(3.4, 2, 0.35);
    // ProfiledPIDController AlignPIDTheta2 = new ProfiledPIDController(.15, 0, 0.01, new TrapezoidProfile.Constraints(DriveConstants.MaxAngularRate, 12));
    double rateLimit = 9.5;
    SlewRateLimiter xlimit = new SlewRateLimiter(rateLimit);
    SlewRateLimiter ylimit = new SlewRateLimiter(rateLimit);
    SlewRateLimiter thetalimit = new SlewRateLimiter(rateLimit);


    double X, Y, Theta;

    double AmpSetpointX = 1.73, AmpSetpointY = 7.23, AmpSetpointTheta = -90;
    double TrapSetpoint1X = 4.1, TrapSetpoint1Y = 2.8, TrapSetpoint1Theta = -120;
    double TrapSetpoint2X = 4, TrapSetpoint2Y = 5.2, TrapSetpoint2Theta = 120;
    double TrapSetpoint3X = 6.2, TrapSetpoint3Y = 4, TrapSetpoint3Theta = 0;

    double toleranceX = .02, toleranceY = .02, toleranceTheta = Math.toRadians(3);
    double fieldRelativeOffset;
    boolean red;

    // Optional<Alliance> ally = DriverStation.getAlliance();
    public AutoAlignSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        AlignPIDTheta.enableContinuousInput(-Math.PI, Math.PI);
        // AlignPIDTheta2.enableContinuousInput(-Math.PI, Math.PI);
        AlignPIDTheta.setIntegratorRange(-0.3, 0.3);
        // AlignPIDTheta2.setIntegratorRange(-0.3, 0.3);
        AlignPIDX.setIntegratorRange(-0.45, 0.45);
        AlignPIDY.setIntegratorRange(-0.45, 0.45);
        
        if (true) {
            if (false) {
                //red

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
                //Blue

                //[1.8668397151100247, 7.570062436206427, 92.40526427834128]
                AmpSetpointX = 1.8668397151100247;
                AmpSetpointY = 7.570062436206427;
                AmpSetpointTheta = Math.toRadians(-90);

                TrapSetpoint1X = 4.1;
                TrapSetpoint1Y = 2.8;
                TrapSetpoint1Theta = Math.toRadians(-120);

                TrapSetpoint2X = 4;
                TrapSetpoint2Y = 5.2;
                TrapSetpoint2Theta = Math.toRadians(120);

                TrapSetpoint3X = 6.2;
                TrapSetpoint3Y = 4;
                TrapSetpoint3Theta = Math.toRadians(180);
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
        // Theta = new Rotation2d(drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians()).rotateBy(new Rotation2d(fieldRelativeOffset)).getRadians();
        Theta = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians();

        alignSpeeds = new ChassisSpeeds(
                xlimit.calculate((MathUtil.clamp((AlignPIDY.calculate(X, PIDSetpointX)), -MaxSpeed, MaxSpeed)) * 1), // Velocity X
                ylimit.calculate((MathUtil.clamp((AlignPIDX.calculate(Y, PIDSetpointY)), -MaxSpeed, MaxSpeed)) * 1), // Velocity Y
                thetalimit.calculate(-MathUtil.clamp((AlignPIDTheta.calculate(Theta, (PIDSetpointTheta))), -MaxAngularRate, MaxAngularRate))); // Rotational Speeds
        fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(alignSpeeds,
                new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
                        .rotateBy(new Rotation2d(0)));

    }

    public void AutoAimAmp() { // Works amp
        AutoAim(AmpSetpointX, AmpSetpointY, AmpSetpointTheta, "Amp");
    }

    public void AutoAimTrap() { // Works with all trap locations
        if (true) {
            // red
            if (Y < 4 && X > 10.75) {
                // Trap 1
                AutoAim(TrapSetpoint1X, TrapSetpoint1Y, TrapSetpoint1Theta, "RedTrap1");
            } else if (Y > 4 && X > 10.75) {
                // Trap 2
                AutoAim(TrapSetpoint2X, TrapSetpoint2Y, TrapSetpoint2Theta, "RedTrap2");
            } else if (X < 10.75) {
                // Trap 3
                AutoAim(TrapSetpoint3X, TrapSetpoint3Y, TrapSetpoint3Theta, "RedTrap3");
            }
        } else {
            // blue
            if (Y < 4 && X < 5.3) {
                // Trap 1
                AutoAim(TrapSetpoint1X, TrapSetpoint1Y, TrapSetpoint1Theta, "BlueTrap1");
            } else if (Y > 4 && X < 5.3) {
                // Trap 2
                AutoAim(TrapSetpoint2X, TrapSetpoint2Y, TrapSetpoint2Theta, "BlueTrap2");
            } else if (X > 5.3) {
                // Trap 3
                AutoAim(TrapSetpoint3X, TrapSetpoint3Y, TrapSetpoint3Theta, "BlueTrap3");
            } else {
            }
        }
    }

    public void AutoAim(double SetpointX, double SetpointY, double SetpointTheta, String location) {
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
    public void updateOffset(double fieldRelativeOffset) {
        this.fieldRelativeOffset = fieldRelativeOffset;
    }
}
//     @Override
//     public void periodic() {
//     }
// }
