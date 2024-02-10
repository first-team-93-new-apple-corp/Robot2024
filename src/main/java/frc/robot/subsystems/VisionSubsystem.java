package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-front");
    double tx, ty, tl, ta, tid, targetpose_robotspace;
    Pose2d pose;
    double LimelightAngle = 29.8;
    SwerveDriveSubsystem m_DriveSubsystem = TunerConstants.DriveTrain;
    Telemetry m_Telemetry;
    SwerveDriveState state;
    double[] LimelightValue = new double[4];
    double[] LimelightTrapValue = new double[4];
    Joystick joystick1 = new Joystick(1);
    public final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;
    private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public VisionSubsystem() {

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
        targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDouble(0);
    }

    public boolean hasTargets() {
        updateValues();
        return ta > 0;
    }

    public void AutoAimTrap() {
        updateValues();
        Optional<Alliance> DS = DriverStation.getAlliance();
        if (hasTargets()) {
            if (DS.get() == Alliance.Red) {
                // if (joystick1.getRawButton(Constants.Thrustmaster.Center_Button)) {
                //     // auto aim to trap
                //     if (tid == 16 || tid == 15 || tid == 14) {
                //         // see trap
                //         m_DriveSubsystem.setControl(m_driveRequest.withRotationalRate(tx / 29.8));
                //     }
                // }
            }
            if (DS.get() == Alliance.Blue) {
                if (joystick1.getRawButton(Constants.Thrustmaster.Center_Button)) {
                    // auto aim to trap
                    if (tid == 16 || tid == 15 || tid == 14) {
                        // see trap
                        // m_DriveSubsystem.applyRequest(m_driveRequest.withRotationalRate(tx / 29.8));
                    }
                }
            }
        }
    }

    public void AutoAimAmp() {
        updateValues();
        Optional<Alliance> DS = DriverStation.getAlliance();
        if (hasTargets()) {
            // if (DS.get() == Alliance.Red) {
            //     // auto aim to amp
            //     if (tid == 6) {
            //         // see amp
            //         m_DriveSubsystem.setControl(m_driveRequest.withRotationalRate(tx / 29.8));
            //     }
            // }
            if (DS.get() == Alliance.Blue) {
                // auto aim to amp
                if (tid == 6) {
                    // see amp
                    AimAmp();
                }
            }
        }
    }
    public void AimAmp(){
        if (tx > 0){
            LimelightAngle = -LimelightAngle;
        } else if (tx < 0){
            LimelightAngle = LimelightAngle;
        } else {
            return;
        }
        m_DriveSubsystem.applyRequest(() -> m_driveRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(tx * LimelightAngle));
    }

    public void gotoTrap(double[] LimelightTrapValue) {
        double Ttx = -3.6665;
        double Tty = 20.84;
        double Tta = 2.61;
        double Ttl = 22 - 25;
        LimelightTrapValue[0] = Ttx;
        LimelightTrapValue[1] = Tty;
        LimelightTrapValue[2] = Ttl;
        LimelightTrapValue[3] = Tta;
    }

    public Pose2d getPose2d() {
        return pose;
    }

    @Override
    public void periodic() {
        updateValues();
        LimelightValue[0] = tx;
        LimelightValue[1] = ty;
        LimelightValue[2] = tl;
        LimelightValue[3] = ta;
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
