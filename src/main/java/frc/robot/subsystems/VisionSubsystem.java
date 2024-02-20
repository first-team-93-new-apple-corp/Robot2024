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

    ChassisSpeeds alignSpeeds, rotateAlignSpeeds, xAlignSpeeds, yAlignSpeeds, BlankSpeeds; // Chassis Speeds which
                                                                                                // robot uses for
    // auto align

    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

    PIDController AlignPIDX = new PIDController(.1, 0, 0); // Rotationly PID
    PIDController AlignPIDY = new PIDController(.1, 0, 0);
    PIDController AlignRotate = new PIDController(.1, 0, 0);

    double tx, ty, tl, ta, tid, ts;
    double[] targetpose_robotspace, botpose;
    double x, y, z;
    double TrapAlignY = 3.3; 
    double TrapAlignX = -18.4;
    double TrapAlignRotate = 0;
    double arcSpeed;
    double tv;

    public VisionSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        // tl = m_limelight.getEntry("tl").getDouble(0);
        tv = m_limelight.getEntry("tv").getDouble(0);
        ta = m_limelight.getEntry("ta").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        ts = m_limelight.getEntry("ts").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }

    public void updateValues() {
        arcSpeed = -MathUtil.clamp((AlignRotate.calculate(ts, TrapAlignRotate)), -MaxAngularRate, MaxAngularRate);
        tx = m_limelight.getEntry("tx").getDouble(0);
        tv = m_limelight.getEntry("tv").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        ts = m_limelight.getEntry("ts").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        y = targetpose_robotspace[2];
        arcSpeed = -MathUtil.clamp((AlignPIDX.calculate(ts, TrapAlignRotate)), -MaxAngularRate, MaxAngularRate);
        if (ts >= 0 && ts <= 13) {
            arcSpeed = -arcSpeed;
        } else {
            arcSpeed = arcSpeed;
        }
        rotateAlignSpeeds = new ChassisSpeeds(
                (0),
                (0),
                (arcSpeed));
        xAlignSpeeds = new ChassisSpeeds(
                (0),
                (-MathUtil.clamp((AlignPIDX.calculate(tx, TrapAlignX)), -MaxSpeed / 5, MaxSpeed / 5)),
                (0));
        yAlignSpeeds = new ChassisSpeeds(
                (-MathUtil.clamp((AlignPIDY.calculate(ty, TrapAlignY)), -MaxSpeed / 5, MaxSpeed / 5)),
                (0),
                (0));
        BlankSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    // (-MathUtil.clamp((AlignPID.calculate(ty, PIDSetpointY)), 0, MaxSpeed / 3)),
    // Velocity y CONNOr MathUtil.clamp((AlignPIDY.calculate(tx,
    // PIDSetpointRotate2)), -MaxSpeed/5, MaxSpeed/5)
    // (-MathUtil.clamp((AlignPID.calculate(tx, PIDSetpointRotate)),
    // -MaxAngularRate, MaxAngularRate))); // Rotational Speeds
    // (-MathUtil.clamp((AlignPID.calculate(y, PIDSetpointY2)), 0, MaxSpeed / 3)),
    // Velocity y Sawyer
    // -MathUtil.clamp((AlignPID.calculate(z2, PIDSetpointZ3)), -MaxAngularRate / 2,
    // MaxAngularRate / 2)
    // (-MathUtil.clamp((AlignPID.calculate(y, PIDSetpointY2)), -MaxSpeed/3 ));
    // Velocity y SAWYEERS THING
    public boolean hasTargets() {
        updateValues();
        if (tv == 1) {
            return true;
        } else {
            return false;
        }
    }

    public enum AutoAlignTrap {
        stage1,
        stage2,
        stage3
    }

    public AutoAlignTrap Curentstate = AutoAlignTrap.stage1;

    public void Align() {
        switch (Curentstate) {
            default:
            case stage1:
                RotateAlign();
                break;
            case stage2:
                XAlign();
                break;
            case stage3:
                YAlign();
                break;
        }
    }

    public void resetState() {
        Curentstate = AutoAlignTrap.stage1;
    }

    public void XAlign() { // Works amp
        if (hasTargets()) {
            if (-MathUtil.clamp((AlignPIDX.calculate(tx, TrapAlignX)), -MaxSpeed / 5, MaxSpeed / 5) <= 0.2
                    && -MathUtil.clamp((AlignPIDX.calculate(tx, TrapAlignX)), -MaxSpeed / 5,
                            MaxSpeed / 5) >= -0.2) {
                Curentstate = AutoAlignTrap.stage3;
            } else
                drivetrain.driveRobotRelative(xAlignSpeeds);
        } else {
            drivetrain.driveRobotRelative(BlankSpeeds);
        }
    }

    public void YAlign() { // Works with all trap locations
        if (hasTargets()) {
            if (-MathUtil.clamp((AlignPIDY.calculate(ty, TrapAlignY)), -MaxSpeed / 5, MaxSpeed / 5) <= 0.2
                    && -MathUtil.clamp((AlignPIDY.calculate(ty, TrapAlignY)), -MaxSpeed / 5, MaxSpeed / 5) >= -0.2) {
                Curentstate = AutoAlignTrap.stage1;
            } else {
                drivetrain.driveRobotRelative(yAlignSpeeds);
            }
        } else {
            drivetrain.driveRobotRelative(BlankSpeeds);
        }
    }

    public void RotateAlign() {
        if (hasTargets()) {
            if (arcSpeed <= 0.2 && arcSpeed >= -0.2) {
                Curentstate = AutoAlignTrap.stage2;
            } else {
                drivetrain.driveRobotRelative(rotateAlignSpeeds);
            }
        } else {
            drivetrain.driveRobotRelative(BlankSpeeds);
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
