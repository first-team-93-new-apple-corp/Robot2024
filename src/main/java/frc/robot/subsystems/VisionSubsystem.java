// package frc.robot.subsystems;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class VisionSubsystem extends SubsystemBase {
//     public final double MaxSpeed = DriveConstants.MaxSpeed / 3;
//     public final double MaxAngularRate = DriveConstants.MaxAngularRate / 3;

//     SwerveDriveSubsystem drivetrain;

//     ChassisSpeeds rotateAlignSpeeds, xAlignTrapSpeeds, yAlignTrapSpeeds, xAlignAmpSpeeds, yAlignAmpSpeeds, BlankSpeeds;

//     NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

//     PIDController AlignPIDX = new PIDController(.05, 0, 0);
//     PIDController AlignPIDY = new PIDController(.1, 0, 0);
//     PIDController AlignRotate = new PIDController(.05, 0, 0.029);

//     double tx, ty, tl, ta, tid, ts;
//     double[] targetpose_robotspace, botpose;
//     double x, y, z;
//     double TrapAlignSetpointY = 3.3;
//     double TrapAlignSetpointX = -18.4;
//     double AmpAlignSetpointY = 15;
//     double AmpAlignSetpointX = -13.5;
//     double AlignRotateSetpoint = 0;
//     double arcSpeedTrap, arcSpeedAmp, xTrapSpeed, yTrapSpeed, xAmpSpeed, yAmpSpeed;
//     double tv;

//     public VisionSubsystem(SwerveDriveSubsystem drivetrain) {
//         this.drivetrain = drivetrain;
//         tx = m_limelight.getEntry("tx").getDouble(0);
//         ty = m_limelight.getEntry("ty").getDouble(0);
//         tv = m_limelight.getEntry("tv").getDouble(0);
//         ta = m_limelight.getEntry("ta").getDouble(0);
//         tid = m_limelight.getEntry("tid").getDouble(0);
//         ts = m_limelight.getEntry("ts").getDouble(0);
//         targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
//     }

//     public boolean hasTargets() {
//         updateValues();
//         if (tv == 1) {
//             return true;
//         } else
//             return false;
//     }

//     public enum AutoAlignTrap {
//         rotateTrap, XTrap, YTrap
//     }

//     public enum AutoAlignAmp {
//         rotateAmp, XAmp, YAmp
//     }

//     public AutoAlignTrap CurentstateTrap = AutoAlignTrap.rotateTrap;

//     public AutoAlignAmp CurentstateAmp = AutoAlignAmp.rotateAmp;

//     public void AlignTrap() {
//         switch (CurentstateTrap) {
//             default:
//             case rotateTrap:
//                 RotateAlignTrap();
//             case XTrap:
//                 XAlignTrap();
//             case YTrap:
//                 YAlignTrap();
//         }
//     }

//     public void AlignAmp() {
//         switch (CurentstateAmp) {
//             default:
//             case rotateAmp:
//                 RotateAlignAmp();
//             case XAmp:
//                 XAlignAmp();
//             case YAmp:
//                 YAlignAmp();
//         }
//     }

//     public void resetStateTrap() {
//         CurentstateTrap = AutoAlignTrap.rotateTrap;
//     }

//     public void resetStateAmp() {
//         CurentstateAmp = AutoAlignAmp.rotateAmp;
//     }

//     public void XAlignTrap() {
//         if (hasTargets()) {
//             if (xTrapSpeed <= 0.2 && xTrapSpeed >= -0.2) {
//                 CurentstateTrap = AutoAlignTrap.YTrap;
//             } else
//                 drivetrain.driveRobotRelative(xAlignTrapSpeeds);
//         } else {
//             drivetrain.driveRobotRelative(BlankSpeeds);
//         }
//     }

//     public void YAlignTrap() {
//         if (hasTargets()) {
//             if (yTrapSpeed <= 0.2 && yTrapSpeed >= -0.2) {
//                 CurentstateTrap = AutoAlignTrap.rotateTrap;
//             } else {
//                 drivetrain.driveRobotRelative(yAlignTrapSpeeds);
//             }
//         } else {
//             drivetrain.driveRobotRelative(BlankSpeeds);
//         }
//     }

//     public void XAlignAmp() {
//         if (hasTargets()) {
//             if (xAmpSpeed <= 0.2 && xAmpSpeed >= -0.2) {
//                 CurentstateAmp = AutoAlignAmp.YAmp;
//             } else
//                 drivetrain.driveRobotRelative(xAlignAmpSpeeds);
//         } else {
//             drivetrain.driveRobotRelative(BlankSpeeds);
//         }
//     }

//     public void YAlignAmp() {
//         if (hasTargets()) {
//             if (yAmpSpeed <= 0.2 && yAmpSpeed >= -0.2) {
//                 CurentstateAmp = AutoAlignAmp.rotateAmp;
//             } else {
//                 drivetrain.driveRobotRelative(yAlignAmpSpeeds);
//             }
//         } else {
//             drivetrain.driveRobotRelative(BlankSpeeds);
//         }
//     }

//     public void RotateAlignAmp() {
//         if (hasTargets()) {
//             if (arcSpeedAmp <= 0.2 && arcSpeedAmp >= -0.2) {
//                 CurentstateTrap = AutoAlignTrap.XTrap;
//                 CurentstateAmp = AutoAlignAmp.XAmp;
//             } else {
//                 drivetrain.driveRobotRelative(rotateAlignSpeeds);
//             }
//         } else {
//             drivetrain.driveRobotRelative(BlankSpeeds);
//         }
//     }

//     public void RotateAlignTrap() {
//         if (hasTargets()) {
//             if (arcSpeedTrap <= 0.2 && arcSpeedTrap >= -0.2) {
//                 CurentstateTrap = AutoAlignTrap.XTrap;
//                 CurentstateAmp = AutoAlignAmp.XAmp;
//             } else {
//                 drivetrain.driveRobotRelative(rotateAlignSpeeds);
//             }
//         } else {
//             drivetrain.driveRobotRelative(BlankSpeeds);
//         }
//     }

//     public void setAlignSpeeds() {
//         if (ts >= 0 && ts <= 40) {
//             // arcSpeed = MathUtil.clamp((AlignRotate.calculate(ts, AlignRotateSetpoint)), -MaxAngularRate,
//                     MaxAngularRate);
//         } else {
//             arcSpeed = -MathUtil.clamp((AlignRotate.calculate(ts, AlignRotateSetpoint)), -MaxAngularRate,
//                     MaxAngularRate);
//         }
//         rotateAlignSpeeds = new ChassisSpeeds(
//                 (0),
//                 (0),
//                 (arcSpeed));
//         xAlignTrapSpeeds = new ChassisSpeeds(
//                 (0),
//                 (xTrapSpeed),
//                 (0));
//         yAlignTrapSpeeds = new ChassisSpeeds(
//                 (yTrapSpeed),
//                 (0),
//                 (0));
//         xAlignAmpSpeeds = new ChassisSpeeds(
//                 (0),
//                 (xAmpSpeed),
//                 (0));
//         yAlignAmpSpeeds = new ChassisSpeeds(
//                 (yAmpSpeed),
//                 (0),
//                 (0));
//         BlankSpeeds = new ChassisSpeeds(
//                 (0),
//                 (0),
//                 (0));
//         xTrapSpeed = -MathUtil.clamp((AlignPIDX.calculate(tx, TrapAlignSetpointX)), -MaxSpeed / 3, MaxSpeed / 3);
//         yTrapSpeed = -MathUtil.clamp((AlignPIDY.calculate(ty, TrapAlignSetpointY)), -MaxSpeed / 3, MaxSpeed / 3);
//         xAmpSpeed = -MathUtil.clamp((AlignPIDX.calculate(tx, AmpAlignSetpointX)), -MaxSpeed / 3, MaxSpeed / 3);
//         yAmpSpeed = -MathUtil.clamp((AlignPIDY.calculate(ty, AmpAlignSetpointY)), -MaxSpeed / 4, MaxSpeed / 4);
//     }

//     public void LimeLightOn() {
//         m_limelight.getEntry("ledMode").setNumber(3);
//     }

//     public void LimeLightOff() {
//         m_limelight.getEntry("ledMode").setNumber(0);
//     }

//     public void updateValues() {
//         setAlignSpeeds();
//         tx = m_limelight.getEntry("tx").getDouble(0);
//         tv = m_limelight.getEntry("tv").getDouble(0);
//         ty = m_limelight.getEntry("ty").getDouble(0);
//         tid = m_limelight.getEntry("tid").getDouble(0);
//         ts = m_limelight.getEntry("ts").getDouble(0);
//         targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
//         y = targetpose_robotspace[2];
//     }

//     @Override
//     public void periodic() {
//         updateValues();
//         SmartDashboard.putBoolean("Has targets", hasTargets());
//         SmartDashboard.putNumber("y", y);
//         SmartDashboard.putNumber("yAmpSpeeds", yAmpSpeed);
//         SmartDashboard.putNumber("xAmpSpeeds", xAmpSpeed);
//         SmartDashboard.putNumber("yTrapSpeeds", yTrapSpeed);
//         SmartDashboard.putNumber("xTrapSpeeds", xTrapSpeed);
//         SmartDashboard.putNumber("rSpeed", arcSpeed);
//     }
// }
