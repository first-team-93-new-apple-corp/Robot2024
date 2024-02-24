package frc.robot.subsystems;

import java.util.Currency;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

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

    ChassisSpeeds rotateAlignSpeeds, xAlignSpeeds, yAlignSpeeds, blankSpeeds;

    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

    PIDController AlignPIDX = new PIDController(.05, 0, 0);
    PIDController AlignPIDY = new PIDController(.1, 0, 0);
    PIDController AlignRotate = new PIDController(.05, 0, 0.029);

    double tx, ty, tl, ta, tid, ts;
    double[] targetpose_robotspace, botpose;
    double x, y, z;
    double TrapAlignSetpointY = 3.3;
    double TrapAlignSetpointX = -18.4;
    double AmpAlignSetpointY = 15;
    double AmpAlignSetpointX = -13.5;
    double AlignRotateSetpoint = 0;
    double arcSpeed, xTrapSpeed, yTrapSpeed, xAmpSpeed, yAmpSpeed;
    double tv;
    double xSpeeds, ySpeeds;

    public VisionSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tv = m_limelight.getEntry("tv").getDouble(0);
        ta = m_limelight.getEntry("ta").getDouble(0);
        tid = m_limelight.getEntry("tid").getDouble(0);
        ts = m_limelight.getEntry("ts").getDouble(0);
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }

    public boolean hasTargets() {
        updateValues();
        if (tv == 1) {
            return true;
        } else
            return false;
    }

    public enum AutoAlign {
        TrapR, TrapX, TrapY, AmpR, AmpY, AmpX, none
    }

    public AutoAlign Curentstate, Curentstate2;

    public void AutoAlign() {
        switch (Curentstate) {
            default:
            case TrapR:
                RotateAlign();
                break;
            case TrapX:
                XAlign();
                break;
            case TrapY:
                YAlign();
                break;
        }
        switch (Curentstate2) {
            default:
            case AmpR:
                RotateAlign2();
                break;
            case AmpX:
                XAlign2();
                break;
            case AmpY:
                YAlign2();
                break;
        }

    }

    public void resetState() {
        Curentstate = AutoAlign.TrapR;
        Curentstate2 = AutoAlign.AmpR;
    }

    public void XAlign() {
        if (hasTargets()) {
            if (xAmpSpeed <= 0.2 && xAmpSpeed >= -0.2) {
                Curentstate = AutoAlign.TrapY;
            } else
                drivetrain.driveRobotRelative(xAlignSpeeds);
        } else {
            drivetrain.driveRobotRelative(blankSpeeds);
        }
    }

    public void YAlign() {
        if (hasTargets()) {
            if (yAmpSpeed <= 0.2 && yAmpSpeed >= -0.2) {
                Curentstate = AutoAlign.TrapR;
            } else {
                drivetrain.driveRobotRelative(yAlignSpeeds);
            }
        } else {
            drivetrain.driveRobotRelative(blankSpeeds);
        }
    }

    public void RotateAlign() {
        if (hasTargets()) {
            if (arcSpeed <= 0.2 && arcSpeed >= -0.2) {
                Curentstate = AutoAlign.TrapX;
            } else {
                drivetrain.driveRobotRelative(rotateAlignSpeeds);
            }
        } else {
            drivetrain.driveRobotRelative(blankSpeeds);
        }
    }

    public void XAlign2() {
        if (hasTargets()) {
            if (xAmpSpeed <= 0.2 && xAmpSpeed >= -0.2) {
                Curentstate = AutoAlign.AmpY;
            } else
                drivetrain.driveRobotRelative(xAlignSpeeds);
        } else {
            drivetrain.driveRobotRelative(blankSpeeds);
        }
    }

    public void YAlign2() {
        if (hasTargets()) {
            if (yAmpSpeed <= 0.2 && yAmpSpeed >= -0.2) {
                Curentstate = AutoAlign.AmpR;
            } else {
                drivetrain.driveRobotRelative(yAlignSpeeds);
            }
        } else {
            drivetrain.driveRobotRelative(blankSpeeds);
        }
    }

    public void RotateAlign2() {
        if (hasTargets()) {
            if (arcSpeed <= 0.2 && arcSpeed >= -0.2) {
                Curentstate = AutoAlign.AmpX;
            } else {
                drivetrain.driveRobotRelative(rotateAlignSpeeds);
            }
        } else {
            drivetrain.driveRobotRelative(blankSpeeds);
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
        if (tid == 11 || tid == 12 || tid == 13 || tid == 14 || tid == 15 || tid == 16) {
            xSpeeds = -MathUtil.clamp((AlignPIDX.calculate(tx, TrapAlignSetpointX)), -MaxSpeed / 3, MaxSpeed / 3);
        } else if (tid == 6 || tid == 5) {
            xSpeeds = -MathUtil.clamp((AlignPIDX.calculate(tx, AmpAlignSetpointX)), -MaxSpeed / 4, MaxSpeed / 4);
        } else {
            xSpeeds = 0;
        }
        if (tid == 11 || tid == 12 || tid == 13 || tid == 14 || tid == 15 || tid == 16) {
            ySpeeds = -MathUtil.clamp((AlignPIDY.calculate(ty, TrapAlignSetpointY)), -MaxSpeed / 3, MaxSpeed / 3);
        } else if (tid == 6 || tid == 5) {
            ySpeeds = -MathUtil.clamp((AlignPIDY.calculate(ty, AmpAlignSetpointY)), -MaxSpeed / 3, MaxSpeed / 3);
        } else {
            ySpeeds = 0;
        }
        xTrapSpeed = xSpeeds;
        yTrapSpeed = xSpeeds;
        xAmpSpeed = ySpeeds;
        yAmpSpeed = ySpeeds;
        rotateAlignSpeeds = new ChassisSpeeds(
                (0),
                (0),
                (arcSpeed));
        yAlignSpeeds = new ChassisSpeeds(
                (ySpeeds),
                (0),
                (0));
        xAlignSpeeds = new ChassisSpeeds(
                (0),
                (xSpeeds),
                (0));
        blankSpeeds = new ChassisSpeeds(
                (0),
                (0),
                (0));
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
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        y = targetpose_robotspace[2];
    }

    @Override
    public void periodic() {
        updateValues();
        SmartDashboard.putBoolean("Has targets", hasTargets());
        SmartDashboard.putNumber("y", y);
    }
}
