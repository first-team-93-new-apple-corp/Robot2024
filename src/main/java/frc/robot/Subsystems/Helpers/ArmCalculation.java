package frc.robot.subsystems.Helpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShoulderSubsystem;

public class ArmCalculation extends SubsystemBase {
    public NetworkTable table;
    double[] values = new double[6];
    private double setpoint;
    ShoulderSubsystem m_ShoulderSubsystem;

    public ArmCalculation() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        m_ShoulderSubsystem = new ShoulderSubsystem();
    }

    public double[] getValues() {
        // This gets the robot position compared to the target
        return NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose_targetspace")
                .getDoubleArray(values);
    }

    /*
     * TO WHOEVER THIS MAY CONCERN:
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     * 
     * To find an equation for the arm, we need to gather a table of distance from
     * the speaker and the angle of the shoulder. To obtain the distance from the
     * speaker, we will use the limelight (not sure which value off the top of my
     * head). The limelight is currently set to only see 1 id for testing. Make sure
     * to change this at some point! The telescope will be all the way
     * in for simplicity. When you have at least 3 points, you can use desmos or
     * chatgpt (which I'd recommend) to find an equation that fits the data. Then,
     * replace the 0 in the below method and you should have working arm
     * calculations!!1!!!1! I would then test this a lot to make sure it works. Next
     * up will be autonomous: make it run the paths and hopefully auto aim.
     * 
     * I really hope someone reads this
     * - Justin
     */
    public void calculateSetpoint() {
        // we need a magic equation to calculate the setpoint...
        setpoint = 0;
        m_ShoulderSubsystem.toSetpoint(setpoint);
    }

    public Command calculate() {
        return this.runOnce(() -> calculateSetpoint());
    }
}
