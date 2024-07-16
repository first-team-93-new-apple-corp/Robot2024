package frc.robot.subsystems.Helpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ArmCalculation extends SubsystemBase{
    public NetworkTable table;
    double[] values = new double[6];
    private double setpoint;
    ShoulderSubsystem m_ShoulderSubsystem;
    public ArmCalculation(ShoulderSubsystem m_ShoulderSubsystem) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.m_ShoulderSubsystem = m_ShoulderSubsystem;
    }

    public double[] getValues() {
        values[0] = table.getEntry("tx").getDouble(0);
        values[1] = table.getEntry("ty").getDouble(0);
        values[2] = table.getEntry("ta").getDouble(0);
        values[3] = table.getEntry("ts").getDouble(0);
        values[4] = table.getEntry("tl").getDouble(0);
        values[5] = table.getEntry("tv").getDouble(0);
        return values;
    }
    public void calculateSetpoint() {
        // we need a magic equation to calculate the setpoint...
        setpoint = 0; 
        m_ShoulderSubsystem.toSetpoint(setpoint);
    }
    public Command calculate(){
        return this.runOnce(() -> calculateSetpoint());
    }
}
