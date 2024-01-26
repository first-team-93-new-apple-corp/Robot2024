package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    double tx, ty, tl, ta;
    Pose2d pose;
    SwerveDriveSubsystem drivetrain;
    public VisionSubsystem(SwerveDriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tl = m_limelight.getEntry("tl").getDouble(0);
        ta = m_limelight.getEntry("ta").getDouble(0);
    }

    public void updateValues() {
        tx = m_limelight.getEntry("tx").getDouble(0);
        ty = m_limelight.getEntry("ty").getDouble(0);
        tl = m_limelight.getEntry("tl").getDouble(0);
        ta = m_limelight.getEntry("ta").getDouble(0);
    }

    public boolean hasTargets() {
        updateValues();
        return ta > 0;
    }
    public Pose2d getPose2d() {
        return pose;
    }
    @Override
    public void periodic() {
        updateValues();
        SmartDashboard.putBoolean("Has targets", hasTargets());

        // if (hasTargets()) {
        //     double[] botpose = m_limelight.getEntry("botpose").getDoubleArray(new double[6]);
        //     pose = new Pose2d(new Translation2d(botpose[0], botpose[1]), new Rotation2d(drivetrain.getRotation3d().getAngle()));
        //     drivetrain.runOnce(() -> drivetrain.resetOdometry(pose));
        //     System.out.println("Updated pose");
        // }
    }
}
