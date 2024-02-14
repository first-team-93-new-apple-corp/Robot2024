package frc.robot.subsystems;

import java.security.ProviderException;
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
    PIDController AlignRotatePID = new PIDController(0.1125, 0, 0); //Rotationly PID
    PIDController AlignXPID = new PIDController(0.1125, 0, 0); // Horizontal PID
    PIDController AlignYPID = new PIDController(0.1125, 0, 0); // Proximity PID
    double tx, ty, tl, ta, tid, targetpose_robotspace, PIDRotate, PIDx, PIDy;
    Pose2d pose;
    double PIDSetpointRotate = 13.5;
    double PIDSetpointX = 13.5;
    double PIDSetpointY = -7;
    double LimelightAngle = 29.8;
    double MaxRotate = DriveConstants.MaxAngularRate;
    SwerveDriveSubsystem drivetrain;
    Telemetry m_Telemetry;
    SwerveDriveState state;
    Joystick m_joystick1 = new Joystick(0);
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
        PIDRotate = tx / DriveConstants.MaxSpeed;
        PIDx = tx / DriveConstants.MaxSpeed;
        PIDy = ta / DriveConstants.MaxSpeed;
        SmartDashboard.putNumber("PIDOutput", AlignRotatePID.calculate(-PIDRotate, PIDSetpointRotate));
        SmartDashboard.putNumber("PIDOutput", AlignRotatePID.calculate(-PIDx, PIDSetpointX));
        SmartDashboard.putNumber("PIDOutput", AlignRotatePID.calculate(PIDy, PIDSetpointY));
        // System.out.println("updateing value");
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDouble(0);
        // System.out.println(AlignPID.calculate(tx, 0));

        alignSpeeds = new ChassisSpeeds(
                ((AlignYPID.calculate(PIDy, PIDSetpointY))), // Velocity X
                (0), // Velocity Y
                ((AlignRotatePID.calculate(-PIDRotate , PIDSetpointRotate)))); // Rotational Speeds
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
    }
}
