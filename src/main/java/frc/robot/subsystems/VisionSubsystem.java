package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
    public final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;

    SwerveDriveSubsystem drivetrain;

    ChassisSpeeds alignSpeeds; //Chassis Speeds which robot uses for auto align

    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

    PIDController AlignPID = new PIDController(0.1125, 0, 0); // Rotationly PID

    double tx, ty, tl, ta, tid, targetpose_robotspace;
    double PIDSetpointRotate = -18; //Setpoint for Rotational value (Limlight offset from center) //TODO Change to Setpoint of this years robot
    double PIDSetpointY = 20; //Setpoint for Horizontal driving (Closeness to apriltag) //TODO Change to Setpoint of this years robot

    public VisionSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        // tl = m_limelight.getEntry("tl").getDouble(0);
        // ta = m_limelight.getEntry("ta").getDouble(0);
        // tid = m_limelight.getEntry("tid").getDouble(0);
        // targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDouble(0);
    }

    public void updateValues() {
        tx = m_limelight.getEntry("tx").getDouble(0); 
        ty = m_limelight.getEntry("ty").getDouble(0);
        // tl = m_limelight.getEntry("tl").getDouble(0);
        // ta = m_limelight.getEntry("ta").getDouble(0);
        // tid = m_limelight.getEntry("tid").getDouble(0);
        // targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDouble(0);

        alignSpeeds = new ChassisSpeeds(
                (-MathUtil.clamp((AlignPID.calculate(ty, PIDSetpointY)), 0, MaxSpeed)), // Velocity y
                (0), // Velocity x
                (-MathUtil.clamp((AlignPID.calculate(tx, PIDSetpointRotate)), 0, MaxAngularRate))); // Rotational Speeds
    }

    public boolean hasTargets() {
        updateValues();
        return ta > 0;
    }

    public void AutoAim() { //Works with all filtered apriltags
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

    @Override
    public void periodic() {
        updateValues();
        SmartDashboard.putBoolean("Has targets", hasTargets());
    }
}
