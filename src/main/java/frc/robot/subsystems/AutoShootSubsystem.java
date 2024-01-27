package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoShootSubsystem extends SubsystemBase{
    NetworkTable networkTable;
    double tx;
    double ty;
    double ta;
    double tv;
    
    public AutoShootSubsystem() {
        networkTable = NetworkTableInstance.getDefault().getTable("Pipeline_Name");
    }
    public boolean hasTargets() {
        return tv ==1;
    }
    public void alignSpeaker() {
        
    }
    public void periodic() {
        tx = networkTable.getValue("tx").getDouble();
        ty = networkTable.getValue("ty").getDouble();
        ta = networkTable.getValue("ta").getDouble();
        tv = networkTable.getValue("tv").getDouble();
    }
}
