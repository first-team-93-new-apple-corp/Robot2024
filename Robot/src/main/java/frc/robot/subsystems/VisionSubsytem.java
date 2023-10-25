package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class VisionSubsytem extends SubsystemBase {
    String limelightName;
    NetworkTable limelight;
    DriveSubsystem m_DriveSubsystem;
    double x, y, z, size = 0;
    boolean fieldRel = false;
    Translation2d COR = DriveConstants.dCenter;
    PIDController turnPID = new PIDController(0.0005, 0, 0);
    PIDController drivePID = new PIDController(0.0005, 0, 0);

    public VisionSubsytem(String limelightName) {
        limelightName = this.limelightName;
        limelight = NetworkTableInstance.getDefault().getTable(limelightName);
        m_DriveSubsystem = new DriveSubsystem();
    }
    public void updateValues() {
        NetworkTableEntry tx = limelight.getEntry("tx");
        NetworkTableEntry ty = limelight.getEntry("ty");
        NetworkTableEntry ta = limelight.getEntry("ta");

        // read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        size = ta.getDouble(0.0);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", size);
    }

    public boolean hasTargets() {
        return false;
    }
    public void followTape() {
        x = drivePID.calculate(size, 1);
        z = turnPID.calculate(y, 0);
        m_DriveSubsystem.drive(x, y, z, fieldRel, COR);
    }
    @Override
    public void periodic() {
        updateValues();
        followTape();
    }
}
