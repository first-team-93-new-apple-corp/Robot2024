package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import javax.swing.text.StyledDocument;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutoAlignSubsystem extends SubsystemBase {
    public final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;
    private SwerveRequest.ApplyChassisSpeeds m_swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
    // private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; //
    // My drivetrain

    SwerveDriveSubsystem drivetrain;

    ChassisSpeeds alignSpeeds; // Chassis Speeds which robot uses for auto align
    public ChassisSpeeds fieldSpeeds;
    private Joystick m_Joystick1;

    PIDController AlignPIDTheta = new PIDController(0.0023, 0.025, 0.0015); // Rotationly PID
    PIDController AlignPIDY = new PIDController(1.15, 0.07, 0.075);
    PIDController AlignPIDX = new PIDController(1.15, 0.07, 0.075);

    double AmpSetpointX = 1.75, AmpSetpointY = 7.6, AmpSetpointTheta = -90;
    double TrapSetpoint1X = 4, TrapSetpoint1Y = 2.7, TrapSetpoint1Theta = 0;
    double TrapSetpoint2X = 4, TrapSetpoint2Y = 2.7, TrapSetpoint2Theta = 0;
    double TrapSetpoint3X = 4, TrapSetpoint3Y = 2.7, TrapSetpoint3Theta = 0;
    double toleranceX = .2, toleranceY = .2, toleranceTheta = 12;

    public AutoAlignSubsystem(SwerveDriveSubsystem drivetrain) {

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                AmpSetpointX = 14.6;
                AmpSetpointY = 7.6;
                AmpSetpointTheta = -90;

            }
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                AmpSetpointX = 1.75;
                AmpSetpointY = 7.6;
                AmpSetpointTheta = -90;

            }
        }

        this.drivetrain = drivetrain;
        AlignPIDTheta.enableContinuousInput(-180, 180);
        AlignPIDTheta.setIntegratorRange(-0.3, 0.3);
        AlignPIDX.setIntegratorRange(-0.3, 0.3);
        AlignPIDY.setIntegratorRange(-0.3, 0.3);
        m_Joystick1 = new Joystick(0);
    }

    public void updateValues(double PIDSetpointX, double PIDSetpointY, Double PIDSetpointTheta) {
        drivetrain.updateOdometry();
        alignSpeeds = new ChassisSpeeds(
                ((MathUtil.clamp((AlignPIDX
                        .calculate(drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getX(), PIDSetpointX)),
                        -MaxSpeed, MaxSpeed)) * 1), // Velocity y Sawyer
                ((MathUtil.clamp((AlignPIDY
                        .calculate(drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getY(), PIDSetpointY)),
                        -MaxSpeed, MaxSpeed)) * 1), // Velocity X Sawyer
                (-MathUtil.clamp((AlignPIDTheta.calculate(
                        drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(),
                        PIDSetpointTheta)), -MaxAngularRate, MaxAngularRate))); // Rotational Speeds
        fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(alignSpeeds,
                new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
                        .rotateBy(new Rotation2d(0)));

    }
    // (-MathUtil.clamp((AlignPID.calculate(ty, PIDSetpointY)), 0, MaxSpeed / 3)),
    // // Velocity y CONNOr
    // (-MathUtil.clamp((AlignPID.calculate(tx, PIDSetpointRotate)),
    // -MaxAngularRate, MaxAngularRate))); // Rotational Speeds
    // (-MathUtil.clamp((AlignPID.calculate(y, PIDSetpointY2)), 0, MaxSpeed / 3)),
    // // Velocity y Sawyer
    // -MathUtil.clamp((AlignPID.calculate(z2, PIDSetpointZ3)), -MaxAngularRate / 2,
    // MaxAngularRate / 2)
    // (-MathUtil.clamp((AlignPID.calculate(y, PIDSetpointY2)), -MaxSpeed/3 )); //
    // Velocity y //SAWYEERS THING

    public void AutoAimAmp() { // Works amp
        AutoAim(AmpSetpointX, AmpSetpointY, AmpSetpointTheta);
    }

    public void AutoAimTrap() { // Works with all trap locations
        if (true) {
            // Trap 1
            AutoAim(TrapSetpoint1X, TrapSetpoint1Y, TrapSetpoint1Theta);
        } else if (true) {
            // Trap 2
            AutoAim(TrapSetpoint2X, TrapSetpoint2Y, TrapSetpoint2Theta);
        } else if (true) {
            // Trap 3
            AutoAim(TrapSetpoint3X, TrapSetpoint3Y, TrapSetpoint3Theta);
        }
    }
    public void AutoAim (double SetpointX, double SetpointY, Double SetpointTheta ) {
        updateValues(SetpointX, SetpointY, SetpointTheta);
        if (
            Math.abs(drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getX() - SetpointX) < toleranceX
            &&
            Math.abs(drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getY() - SetpointY) < toleranceY
            &&
            Math.abs(drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees() + SetpointTheta) < toleranceTheta)
            {
            return;
        } else {
            drivetrain.driveRobotRelative(fieldSpeeds);
        }
    }

    @Override
    public void periodic() {
    }
}
