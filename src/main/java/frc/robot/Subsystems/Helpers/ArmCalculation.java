package frc.robot.subsystems.Helpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ArmCalculation {
    public NetworkTable table;
    double[] values = new double[6];
    private double setpoint;
    public ArmCalculation() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
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
    public double calculateSetpoint() {
        // we need a magic equation to calculate the setpoint...
        setpoint = 0; 
        return setpoint;
    }
}
