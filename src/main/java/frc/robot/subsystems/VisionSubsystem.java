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

    ChassisSpeeds alignSpeeds; // Chassis Speeds which robot uses for auto align

    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

    PIDController AlignPIDTheata = new PIDController(0.1125, 0, 0); // Rotationly PID
    PIDController AlignPIDY = new PIDController(.75, 0, 0.1);

    double tx, ty, tl, ta, tid;
    double[] targetpose_robotspace, botpose;
    double x, y, z;
    double PIDSetpointRotate = -13.9; // Setpoint for Rotational value (Limlight offset from center)
    double PIDSetpointY = .1; // Setpoint for Horizontal driving (Closeness to apriltag) 
    double PIDSetpointY2 = .4;
    double PIDSetpointRotate2 = -17.5;
    public VisionSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        // tl = m_limelight.getEntry("tl").getDouble(0);
        ta = m_limelight.getEntry("ta").getDouble(0);
        // tid = m_limelight.getEntry("tid").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }

    public void updateValues() {
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        y = targetpose_robotspace[2];
        if (tid == 6) {
            alignSpeeds = new ChassisSpeeds(
                ((MathUtil.clamp((AlignPIDY.calculate(y, PIDSetpointY)), -MaxSpeed, MaxSpeed))), // Velocity y Sawyer
                (0), // Velocity x
                (-MathUtil.clamp((AlignPIDTheata.calculate(tx, PIDSetpointRotate)), -MaxAngularRate, MaxAngularRate))); // Rotational Speeds
        } else if (tid == 11 || tid == 12 || tid == 13 || tid == 14 || tid == 15 || tid == 16){
            alignSpeeds = new ChassisSpeeds(
                ((MathUtil.clamp((AlignPIDY.calculate(y, PIDSetpointY2)), -MaxSpeed, MaxSpeed))), // Velocity y Sawyer
                (0), // Velocity x
                (-MathUtil.clamp((AlignPIDTheata.calculate(tx, PIDSetpointRotate2)), -MaxAngularRate, MaxAngularRate))); // Rotational Speeds
        } else {
            alignSpeeds = new ChassisSpeeds(
                (0),
                (0),
                (0));
        }
        
}
    // (-MathUtil.clamp((AlignPID.calculate(ty, PIDSetpointY)), 0, MaxSpeed / 3)), // Velocity y CONNOr
    // (-MathUtil.clamp((AlignPID.calculate(tx, PIDSetpointRotate)), -MaxAngularRate, MaxAngularRate))); // Rotational Speeds
    //(-MathUtil.clamp((AlignPID.calculate(y, PIDSetpointY2)), 0, MaxSpeed / 3)), // Velocity y Sawyer
    // -MathUtil.clamp((AlignPID.calculate(z2, PIDSetpointZ3)), -MaxAngularRate / 2, MaxAngularRate / 2)
    // (-MathUtil.clamp((AlignPID.calculate(y, PIDSetpointY2)), -MaxSpeed/3 )); // Velocity y //SAWYEERS THING
    public boolean hasTargets() {
        updateValues();
        return ta > 0;
    }
    public void AutoAimAmp() { // Works amp
        if (hasTargets()) {
            drivetrain.driveRobotRelative(alignSpeeds);
        }
    }
    public void AutoAimTrap(){ // Works with all trap locations
        if (hasTargets()) {
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
        SmartDashboard.putNumber("y", y);
    }
}
