package frc.robot.subsystems;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import java.util.Optional;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.DoubleArrayEntry;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

public class AutoAlignSubsystem extends SubsystemBase {
    public final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;
    // private SwerveRequest.ApplyChassisSpeeds m_swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
    // private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; //
    // My drivetrain
    ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    SwerveDriveSubsystem drivetrain;

    ChassisSpeeds alignSpeeds; // Chassis Speeds which robot uses for auto align

    ChassisSpeeds fieldSpeeds;

    PIDController AlignPIDTheta = new PIDController(0.0023, 0.025, 0.0015); // Rotationly PID
    PIDController AlignPIDY = new PIDController(1.15, 0.07, 0.075);
    PIDController AlignPIDX = new PIDController(1.15, 0.07, 0.075);

    double X, Y, Theta;

    double AmpSetpointX = 1.75, AmpSetpointY = 7.6, AmpSetpointTheta = -90;
    double TrapSetpoint1X = 4.1, TrapSetpoint1Y = 2.8, TrapSetpoint1Theta = -120;
    double TrapSetpoint2X = 4, TrapSetpoint2Y = 5.2, TrapSetpoint2Theta = 120;
    double TrapSetpoint3X = 6.2, TrapSetpoint3Y = 4, TrapSetpoint3Theta = 0;

    double toleranceX = .2, toleranceY = .2, toleranceTheta = 12;

    boolean red;
    Optional<Alliance> ally = DriverStation.getAlliance();
    public AutoAlignSubsystem(SwerveDriveSubsystem drivetrain) {
        SmartDashboard.putBoolean("Red?", red);
        this.drivetrain = drivetrain;
        AlignPIDTheta.enableContinuousInput(-180, 180);
        AlignPIDTheta.setIntegratorRange(-0.3, 0.3);
        AlignPIDX.setIntegratorRange(-0.3, 0.3);
        AlignPIDY.setIntegratorRange(-0.3, 0.3);
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                AmpSetpointX = 14.6;
                AmpSetpointY = 7.6;
                AmpSetpointTheta = -90;
    
                TrapSetpoint1X = 12.5;
                TrapSetpoint1Y = 2.8;
                TrapSetpoint1Theta = -60;
    
                TrapSetpoint2X = 12.6;
                TrapSetpoint2Y = 5.2;
                TrapSetpoint2Theta = 60;
                
                TrapSetpoint3X = 10.4;
                TrapSetpoint3Y = 4;
                TrapSetpoint3Theta = 0;
            }
            if (ally.get() == Alliance.Blue) {
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
        //     AmpSetpointX = 14.6;
        //     AmpSetpointY = 7.6;
        //     AmpSetpointTheta = -90;

        //     TrapSetpoint1X = 12.5;
        //     TrapSetpoint1Y = 2.8;
        //     TrapSetpoint1Theta = -30;

        //     TrapSetpoint2X = 12.6;
        //     TrapSetpoint2Y = 5.2;
        //     TrapSetpoint2Theta = 30;
            
        //     TrapSetpoint3X = 10.4;
        //     TrapSetpoint3Y = 4;
        //     TrapSetpoint3Theta = 0;
        // } else {
        //     AmpSetpointX = 1.75;
        //     AmpSetpointY = 7.6;
        //     AmpSetpointTheta = -90;

        //     TrapSetpoint1X = 4.1;
        //     TrapSetpoint1Y = 2.8;
        //     TrapSetpoint1Theta = -60;

        //     TrapSetpoint2X = 4;
        //     TrapSetpoint2Y = 5.2;
        //     TrapSetpoint2Theta = 60;

        //     TrapSetpoint3X = 6.2;
        //     TrapSetpoint3Y = 4;
        //     TrapSetpoint3Theta = 180;
        // }
    }

    public void updateValues(double PIDSetpointX, double PIDSetpointY, Double PIDSetpointTheta) {
        drivetrain.updateOdometry();
        X = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getX();
        Y = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getY();
        Theta = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees();
        
        alignSpeeds = new ChassisSpeeds(
                ((MathUtil.clamp((AlignPIDX.calculate(X, PIDSetpointX)), -MaxSpeed, MaxSpeed)) * 1), // Velocity X Sawyer
                ((MathUtil.clamp((AlignPIDY.calculate(Y, PIDSetpointY)), -MaxSpeed, MaxSpeed)) * 1), // Velocity Y Sawyer
                (-MathUtil.clamp((AlignPIDTheta.calculate(Theta, PIDSetpointTheta)), -MaxAngularRate, MaxAngularRate))); // Rotational Speeds
                // (-MathUtil.clamp((AlignPIDTheta.calculate(Math.toRadians(Theta), Math.toRadians(PIDSetpointTheta))), -MaxAngularRate, MaxAngularRate))); // Rotational Speeds
        
        fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(alignSpeeds,
            new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
            .rotateBy(new Rotation2d(0)));

    }

    public void AutoAimAmp() { // Works amp
        AutoAim(AmpSetpointX, AmpSetpointY, AmpSetpointTheta);
    }

    public void AutoAimTrap() { // Works with all trap locations
        if (ally.get() == Alliance.Red) {
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
            if (Y < 4 && X < 5.3) {
                 // Trap 1
                 AutoAim(TrapSetpoint1X, TrapSetpoint1Y, TrapSetpoint1Theta);
             } else if (Y > 4 && X < 5.3) {
                // Trap 2
                AutoAim(TrapSetpoint2X, TrapSetpoint2Y, TrapSetpoint2Theta);
            } else if (X> 5.3) {
                // Trap 3
                AutoAim(TrapSetpoint3X, TrapSetpoint3Y, TrapSetpoint3Theta);
            }
        }
    }
    
    public void AutoAim (double SetpointX, double SetpointY, Double SetpointTheta ) {
        updateValues(SetpointX, SetpointY, SetpointTheta);
        
        if (
            Math.abs(X - SetpointX) < toleranceX
            &&
            Math.abs(Y - SetpointY) < toleranceY
            &&
            Math.abs(-180 + Math.abs(Theta) + Math.abs(SetpointTheta)) < toleranceTheta)
            {
            
            // m_ShooterSubsystem.shootAmp();
            return;
        } else {
            drivetrain.driveRobotRelative(fieldSpeeds);
        }
    }

    @Override
    public void periodic() {
    }
}
