package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoAlignSubsystem extends SubsystemBase {
    public final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;
    LEDSubsystem m_LED;
    ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(m_LED);
    // ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    SwerveDriveSubsystem drivetrain;
    ChassisSpeeds alignSpeeds; // Chassis Speeds which robot uses for auto align
    ChassisSpeeds fieldSpeeds;
    PIDController AlignPIDTheta = new PIDController(.25, 0, 0.13); // Rotationly PID
    PIDController AlignPIDX = new PIDController(3.25, 2, 0.4); // Drive PIDs should be the same
    PIDController AlignPIDY = new PIDController(2, 1.5, 0);
    PIDController tofPID = new PIDController(0.008, 0, 0.00125);
    double tofpos, tofposAngle;
    TimeOfFlight tof = new TimeOfFlight(23);
    double rateLimit = 12;
    SlewRateLimiter xlimit = new SlewRateLimiter(rateLimit);
    SlewRateLimiter ylimit = new SlewRateLimiter(rateLimit);
    // SlewRateLimiter thetalimit = new SlewRateLimiter(rateLimit);

    double X, Y, Theta;
    double calculatedX, calculatedY, calculatedTheta;
    double AmpSetpointX = 1.73, AmpSetpointY = 7.23, AmpSetpointTheta = -90;
    double TrapSetpoint1X = 4.1, TrapSetpoint1Y = 2.8, TrapSetpoint1Theta = -120;
    double TrapSetpoint2X = 4, TrapSetpoint2Y = 5.2, TrapSetpoint2Theta = 120;
    double TrapSetpoint3X = 6.2, TrapSetpoint3Y = 4, TrapSetpoint3Theta = 0;

    double toleranceX = .2, toleranceY = .025, toleranceTheta = Math.toRadians(5);
    double fieldRelativeOffset;
    boolean red;
    double tofSetpoint = 293;
    public AutoAlignSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        AlignPIDTheta.enableContinuousInput(-Math.PI, Math.PI);
        // AlignPIDTheta2.enableContinuousInput(-Math.PI, Math.PI);
        AlignPIDTheta.setIntegratorRange(-0.3, 0.3);
        // AlignPIDTheta2.setIntegratorRange(-0.3, 0.3);
        AlignPIDX.setIntegratorRange(-0.4, 0.4);
        AlignPIDY.setIntegratorRange(-0.4, 0.4);
        SmartDashboard.putData("AlignPIDX", AlignPIDX);
        SmartDashboard.putData("AlignPIDY", AlignPIDY);
        SmartDashboard.putData("AlignPIDTheta", AlignPIDTheta); 
        SmartDashboard.putData("tofPID", tofPID);
        tof.setRangingMode(RangingMode.Medium, 24);
        AlignPIDTheta.setTolerance(toleranceTheta);
        AlignPIDX.setTolerance(toleranceX);
        AlignPIDY.setTolerance(toleranceY);

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                // red

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
            } else {
                // Blue

                // [1.8947558534630007, 7.425, 92.89807991692113]
                AmpSetpointX = 1.8947558534630007;
                AmpSetpointY = 7.425;
                AmpSetpointTheta = -Math.PI / 2;

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
        Theta = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians();
        tofpos = tof.getRange();
        if (Theta > 90) {
            tofposAngle = 90-(Theta-90);
        } else {
            tofposAngle = Theta;
        }
        tofpos = tofpos*Math.sin(tofposAngle);
        calculatedX = xlimit
                .calculate((MathUtil.clamp((AlignPIDY.calculate(X, PIDSetpointX)), -MaxSpeed, MaxSpeed)) * 1);
        // calculatedY = ylimit
        //         .calculate((MathUtil.clamp((AlignPIDX.calculate(Y, PIDSetpointY)), -MaxSpeed, MaxSpeed)) * 1);
        calculatedTheta = (-MathUtil.clamp((AlignPIDTheta.calculate(Theta, (PIDSetpointTheta))),
                -MaxAngularRate, MaxAngularRate));
        // if ((calculatedX < toleranceX && calculatedX > -toleranceX)
        //         && (calculatedTheta < toleranceTheta && calculatedTheta > -toleranceTheta)) {
            // calculatedTheta = 0;
            // calculatedX = 0;
            calculatedY = MathUtil.clamp(-tofPID.calculate(tofpos, tofSetpoint), -MaxSpeed, MaxSpeed);
        // }
        // if (AlignPIDX.atSetpoint() && AlignPIDTheta.atSetpoint()) {
            alignSpeeds = new ChassisSpeeds(calculatedX, calculatedY, calculatedTheta);
        // }  else {
        //     alignSpeeds = new ChassisSpeeds(calculatedX, 0, calculatedTheta);
        // }
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                // red
                fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(alignSpeeds,
                new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
                        .rotateBy(new Rotation2d(Math.PI)));
            } else {
                // Blue
                fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(alignSpeeds,
                new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
                        .rotateBy(new Rotation2d(0)));
            }
        }
    }

    public void AutoAimAmp() { // Works amp
        AutoAim(AmpSetpointX, AmpSetpointY, AmpSetpointTheta, "Amp");
    }

    public void AutoAimTrap() { // Works with all trap locations
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
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
    }

    public void AutoAim(double SetpointX, double SetpointY, double SetpointTheta, String location) {
        updateValues(SetpointX, SetpointY, SetpointTheta);

        if (!((AlignPIDX.atSetpoint())
                &
                ((Math.abs(Y - SetpointY) < toleranceY)
                        &
                        (AlignPIDTheta.atSetpoint())))) {
            drivetrain.driveRobotRelative(fieldSpeeds);

        } else {
        }

    }

    public void updateOffset(double fieldRelativeOffset) {
        this.fieldRelativeOffset = fieldRelativeOffset;
    }
}
// @Override
// public void periodic() {
//     SmartDashboard.putNumber("tof distance front bumper", tof.getRange());


// }
// }
