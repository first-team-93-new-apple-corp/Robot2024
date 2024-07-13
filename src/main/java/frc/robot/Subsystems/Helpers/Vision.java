package frc.robot.subsystems.Helpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Vision extends SubsystemBase {
    private NetworkTableEntry limelight = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose-targetspace");
    
    private SwerveDriveSubsystem m_Drive;
    // private Double[] values;
    // private double rotation;
    public Vision(SwerveDriveSubsystem m_Drive) {
        this.m_Drive = m_Drive;
    }
    public void pointTo() {

    }
    @Override
    public void periodic() {
        // values = limelight.getDoubleArray(values);
        // rotation = values[4];
        // SmartDashboard.putNumberArray("limelight values", values);
        // SmartDashboard.putNumber("rotation", rotation);
    }
}
