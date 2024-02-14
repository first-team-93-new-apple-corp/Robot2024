package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    ChassisSpeeds alignSpeeds;
    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-front");
    PIDController AlignPID = new PIDController(0.1125, 0, 0);
    double tx, ty, tl, ta, tid, targetpose_robotspace;
    Pose2d pose;
    double PIDSetpoint = 13.5;
    double LimelightAngle = 29.8;
    double PIDtx;
    SwerveDriveSubsystem drivetrain;
    Telemetry m_Telemetry;
    SwerveDriveState state;
    Joystick m_joystick1 = new Joystick(0);
    double[] LimelightValue = new double[4];
    Joystick joystick1 = new Joystick(1);
    public final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;
    private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public VisionSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tl = m_limelight.getEntry("tl").getDouble(0);
        ta = m_limelight.getEntry("ta").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDouble(0);
    }

    public void updateValues() {
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tl = m_limelight.getEntry("tl").getDouble(0);
        ta = m_limelight.getEntry("ta").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        PIDtx = Math.max(LimelightAngle, targetpose_robotspace)
        SmartDashboard.putNumber("PIDOutput", AlignPID.calculate(-tx, PIDSetpoint/10));
        // System.out.println("updateing value");
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDouble(0);
        // System.out.println(AlignPID.calculate(tx, 0));

        alignSpeeds = new ChassisSpeeds(
                (0), // Velocity X
                (0), // Velocity Y
                ((AlignPID.calculate((-tx / 10) , PIDSetpoint / 10)))); // Rotational Speeds

    }

    public boolean hasTargets() {
        updateValues();
        return ta > 0;
    }

    public void AutoAim() {
        if (hasTargets()) {
            System.out.println("alining");
            drivetrain.driveRobotRelative(alignSpeeds);
        }
    }

    public void LimeLightOn() {
        m_limelight.getEntry("ledMode").setNumber(3);
    }

    public void LimeLightOff() {
        m_limelight.getEntry("ledMode").setNumber(0);
    }

    public Pose2d getPose2d() {
        return pose;
    }

    @Override
    public void periodic() {
        updateValues();
        SmartDashboard.putBoolean("Has targets", hasTargets());
        SmartDashboard.putNumberArray("LimelightValues", LimelightValue);
        // if (hasTargets()) {
        // double[] botpose = m_limelight.getEntry("botpose").getDoubleArray(new
        // double[6]);
        // pose = new Pose2d(new Translation2d(botpose[0], botpose[1]), new
        // Rotation2d(drivetrain.getRotation3d().getAngle()));
        // drivetrain.runOnce(() -> drivetrain.resetOdometry(pose));
        // System.out.println("Updated pose");
        // }
    }
}
