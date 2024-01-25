package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    double tx, ty, tl, ta;
    Pose2d pose;
    public VisionSubsystem() {
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

    @Override
    public void periodic() {
        updateValues();
        SmartDashboard.putBoolean("Has targets", hasTargets());
        if (hasTargets()) {
            // pose = m_limelight.getEntry("botpose");
        }
    }
}
