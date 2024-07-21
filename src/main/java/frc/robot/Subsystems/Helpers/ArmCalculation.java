package frc.robot.subsystems.Helpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ArmCalculation extends SubsystemBase {
    public NetworkTable table;
    double[] values = new double[6];
    double distanceToTarget;
    double tZ;
    double tY;
    private double setpoint;
    ShoulderSubsystem m_ShoulderSubsystem;

    public ArmCalculation(ShoulderSubsystem m_ShoulderSubsystem2) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.m_ShoulderSubsystem = m_ShoulderSubsystem2;
    }

    public double[] getValues() {
        // This gets the robot position compared to the target
        return NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose_targetspace")
                .getDoubleArray(values);
    }

    public double calculateSetpoint() {
        values = getValues();
        tY = values[1];
        tZ = values[2];
        distanceToTarget = Math.sqrt(Math.pow(tY, 2) + Math.pow(tZ, 2));
        setpoint = (13.389 * distanceToTarget) - (39.456);
        return setpoint;
    }

    public Command calculate() {
        return this.runOnce(() -> calculateSetpoint());
    }
    /*
     * List of working Limelight Distance | Shoulder Angle:
     * 2.5 | -2.0
     * 2.11 | -15.0
     * 3.0 | 1.0
     * 3.1 | 2.5
     */
}