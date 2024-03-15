package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    ChassisSpeeds rotateAlignSpeeds, xAlignTrapSpeeds, yAlignTrapSpeeds, xAlignAmpSpeeds, yAlignAmpSpeeds, BlankSpeeds;

    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

    PIDController AlignPIDX = new PIDController(.05, 0, 0);
    PIDController AlignPIDY = new PIDController(.1, 0, 0);
    PIDController AlignRotate = new PIDController(.05, 0, 0.029);

    double tx, ty, ta, tid, ts, tl, cl;
    double[] targetpose_robotspace, botpose, botpose_wpiblue;
    double x, y, z;
    double TrapAlignSetpointY = 3.3;
    double TrapAlignSetpointX = -18.4;
    double AmpAlignSetpointY = 15;
    double AmpAlignSetpointX = -13.5;
    double AlignRotateSetpoint = 0;
    double arcSpeed, xTrapSpeed, yTrapSpeed, xAmpSpeed, yAmpSpeed;
    double tv;

    Pose2d pose;

    public VisionSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tv = m_limelight.getEntry("tv").getDouble(0);
        ta = m_limelight.getEntry("ta").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        ts = m_limelight.getEntry("ts").getDouble(0);
        tl = m_limelight.getEntry("tl").getDouble(0);
        cl = m_limelight.getEntry("cl").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        botpose_wpiblue = m_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        pose = new Pose2d();
    }

    public boolean hasTargets() {
        updateValues();
        if (tv == 1) {
            return true;
        } else
            return false;
    }

    public enum AutoAlignTrap {
        rotateTrap, XTrap, YTrap
    }

    public enum AutoAlignAmp {
        rotateAmp, XAmp, YAmp
    }

    public AutoAlignTrap CurentstateTrap = AutoAlignTrap.rotateTrap;

    public AutoAlignAmp CurentstateAmp = AutoAlignAmp.rotateAmp;

    public void AlignTrap() {
        if (tid == 11 || tid == 12 || tid == 13 || tid == 14 || tid == 15 || tid == 16) {
            switch (CurentstateTrap) {
                default:
                case rotateTrap:
                    RotateAlign();
                    break;
                case XTrap:
                    XAlignTrap();
                    break;
                case YTrap:
                    YAlignTrap();
                    break;
            }
        }
    }

    public void AlignAmp() {
        if (tid == 6 || tid == 5) {
            switch (CurentstateAmp) {
                default:
                case rotateAmp:
                    RotateAlign();
                    break;
                case XAmp:
                    XAlignAmp();
                    break;
                case YAmp:
                    YAlignAmp();
                    break;
            }
        }
    }

    public void resetStateTrap() {
        CurentstateTrap = AutoAlignTrap.rotateTrap;
    }

    public void resetStateAmp() {
        CurentstateAmp = AutoAlignAmp.rotateAmp;
    }

    public void XAlignTrap() {
        if (hasTargets()) {
            if (xTrapSpeed <= 0.2 && xTrapSpeed >= -0.2) {
                CurentstateTrap = AutoAlignTrap.YTrap;
            } else
                drivetrain.driveRobotRelative(xAlignTrapSpeeds);
        } else {
            drivetrain.driveRobotRelative(BlankSpeeds);
        }
    }

    public void YAlignTrap() {
        if (hasTargets()) {
            if (yTrapSpeed <= 0.2 && yTrapSpeed >= -0.2) {
                CurentstateTrap = AutoAlignTrap.rotateTrap;
            } else {
                drivetrain.driveRobotRelative(yAlignTrapSpeeds);
            }
        } else {
            drivetrain.driveRobotRelative(BlankSpeeds);
        }
    }

    public void XAlignAmp() {
        if (hasTargets()) {
            if (xAmpSpeed <= 0.2 && xAmpSpeed >= -0.2) {
                CurentstateAmp = AutoAlignAmp.YAmp;
            } else
                drivetrain.driveRobotRelative(xAlignAmpSpeeds);
        } else {
            drivetrain.driveRobotRelative(BlankSpeeds);
        }
    }

    public void YAlignAmp() {
        if (hasTargets()) {
            if (yAmpSpeed <= 0.2 && yAmpSpeed >= -0.2) {
                CurentstateAmp = AutoAlignAmp.rotateAmp;
            } else {
                drivetrain.driveRobotRelative(yAlignAmpSpeeds);
            }
        } else {
            drivetrain.driveRobotRelative(BlankSpeeds);
        }
    }

    public void RotateAlign() {
        if (hasTargets()) {
            if (arcSpeed <= 0.2 && arcSpeed >= -0.2) {
                CurentstateTrap = AutoAlignTrap.XTrap;
                CurentstateAmp = AutoAlignAmp.XAmp;
            } else {
                drivetrain.driveRobotRelative(rotateAlignSpeeds);
            }
        } else {
            drivetrain.driveRobotRelative(BlankSpeeds);
        }
    }

    public void setAlignSpeeds() {
        if (ts >= 0 && ts <= 40) {
            arcSpeed = MathUtil.clamp((AlignRotate.calculate(ts, AlignRotateSetpoint)), -MaxAngularRate,
                    MaxAngularRate);
        } else {
            arcSpeed = -MathUtil.clamp((AlignRotate.calculate(ts, AlignRotateSetpoint)), -MaxAngularRate,
                    MaxAngularRate);
        }
        rotateAlignSpeeds = new ChassisSpeeds(
                (0),
                (0),
                (arcSpeed));
        xAlignTrapSpeeds = new ChassisSpeeds(
                (0),
                (-MathUtil.clamp((AlignPIDX.calculate(tx, TrapAlignSetpointX)), -MaxSpeed / 3, MaxSpeed / 3)),
                (0));
        yAlignTrapSpeeds = new ChassisSpeeds(
                (-MathUtil.clamp((AlignPIDY.calculate(ty, TrapAlignSetpointY)), -MaxSpeed / 3, MaxSpeed / 3)),
                (0),
                (0));
        xAlignAmpSpeeds = new ChassisSpeeds(
                (0),
                (-MathUtil.clamp((AlignPIDX.calculate(tx, AmpAlignSetpointX)), -MaxSpeed / 4, MaxSpeed / 4)),
                (0));
        yAlignAmpSpeeds = new ChassisSpeeds(
                (-MathUtil.clamp((AlignPIDY.calculate(ty, AmpAlignSetpointY)), -MaxSpeed / 3, MaxSpeed / 3)),
                (0),
                (0));
        BlankSpeeds = new ChassisSpeeds(
                (0),
                (0),
                (0));
        xTrapSpeed = -MathUtil.clamp((AlignPIDX.calculate(tx, TrapAlignSetpointX)), -MaxSpeed / 3, MaxSpeed / 3);
        yTrapSpeed = -MathUtil.clamp((AlignPIDY.calculate(ty, TrapAlignSetpointY)), -MaxSpeed / 3, MaxSpeed / 3);
        xAmpSpeed = -MathUtil.clamp((AlignPIDX.calculate(tx, AmpAlignSetpointX)), -MaxSpeed / 3, MaxSpeed / 3);
        yAmpSpeed = -MathUtil.clamp((AlignPIDY.calculate(ty, AmpAlignSetpointY)), -MaxSpeed / 4, MaxSpeed / 4);
    }

    public void LimeLightOn() {
        m_limelight.getEntry("ledMode").setNumber(3);
    }

    public void LimeLightOff() {
        m_limelight.getEntry("ledMode").setNumber(0);
    }

    public void updateValues() {
        setAlignSpeeds();
        tx = m_limelight.getEntry("tx").getDouble(0);
        tv = m_limelight.getEntry("tv").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        ts = m_limelight.getEntry("ts").getDouble(0);
        tl = m_limelight.getEntry("tl").getDouble(0);
        cl = m_limelight.getEntry("cl").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        botpose_wpiblue = m_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        y = targetpose_robotspace[2];
    }   

    public Pose2d getPose() {
        updateValues();
        pose = new Pose2d(botpose_wpiblue[0], botpose_wpiblue[1], new Rotation2d(Math.toRadians(botpose_wpiblue[5])));
        return pose;
    }

    @Override
    public void periodic() {
        updateValues();
        SmartDashboard.putBoolean("Has targets", hasTargets());
        SmartDashboard.putNumber("y", y);
    }
}