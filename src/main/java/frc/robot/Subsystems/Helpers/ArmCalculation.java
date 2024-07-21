package frc.robot.subsystems.Helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public void calculateSetpoint() {
        values = getValues();
        tY = values[1];
        tZ = values[2];
        distanceToTarget = Math.sqrt(Math.pow(tY, 2) + Math.pow(tZ, 2));
        SmartDashboard.putNumber("Theoretical distance", distanceToTarget);
        setpoint = (16.1086 * distanceToTarget) - (46.5057);
        setpoint = MathUtil.clamp(setpoint, -10, 30);
        m_ShoulderSubsystem.toSetpoint(setpoint);
        // return setpoint;
    }

    public Command calculate() {
        return this.runOnce(() -> calculateSetpoint());
    }
    /*
     * List of working Limelight Distance | Shoulder Angle:
     * 3.0 | -0.5
     * 
     */
}