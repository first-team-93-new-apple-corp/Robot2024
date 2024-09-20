package frc.robot.subsystems.Helpers;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ArmCalculation extends SubsystemBase {
    public NetworkTable table;
    double[] values = new double[6];
    double distanceToTarget;
    double tZ;
    double tY;
    double a;
    double b;
    double c;
    double unclamped;
    double angleRad;
    // deltaX^2 + deltaY^2 + deltaZ^2 = Distance^2!
    private double setpoint;
    ShoulderSubsystem m_ShoulderSubsystem;
    public Supplier<Pose2d> pose;

    public ArmCalculation(ShoulderSubsystem m_ShoulderSubsystem2, Supplier<Pose2d> pose) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.m_ShoulderSubsystem = m_ShoulderSubsystem2;
        this.pose = pose;
    }

    public double[] getValues() {
        // This gets the robot position compared to the target
        return NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose_targetspace")
                .getDoubleArray(values);
    }

    public void calculateSetpoint() {
        // values = getValues();
        // tY = values[1];
        // tZ = values[2];
        // distanceToTarget = Math.sqrt(Math.pow(tY, 2) + Math.pow(tZ, 2));
        // SmartDashboard.putNumber("Theoretical distance", distanceToTarget);
        // setpoint = (16.1086 * distanceToTarget) - (46.5057);
        // distanceToTarget = (((pose.get().getX() - Constants.goals.RedSpeaker.getX())
        //                     *(pose.get().getX() - Constants.goals.RedSpeaker.getX()))
        //                     +(((pose.get().getY() - Constants.goals.RedSpeaker.getY()))
        //                     *((pose.get().getY() - Constants.goals.RedSpeaker.getY())))
        //                     +(((0 - Constants.goals.RedSpeaker.getZ()))
        //                     *((0 - Constants.goals.RedSpeaker.getZ()))));
        b = (pose.get().getX() - Constants.goals.RedSpeaker.getX());
        a = (Constants.goals.RedSpeaker.getZ());
        c = ((a*a)+(b*b));
        // d = ((a*a)+(b*b)+(c*c))
        distanceToTarget = Math.sqrt(c);

        setpoint = Math.acos(b/(distanceToTarget));
        angleRad = setpoint;
        setpoint = Math.toDegrees(angleRad)+45-20-12;
        unclamped = (setpoint);
        setpoint = MathUtil.clamp(setpoint, -10, 30);
        m_ShoulderSubsystem.toSetpoint(setpoint);
    }

    public Command calculate() {
        return this.runOnce(() -> calculateSetpoint());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("distancetotarget", distanceToTarget);
        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putNumber("unclamped Setpoint", unclamped);
        SmartDashboard.putNumber("getx", pose.get().getX()- Constants.goals.RedSpeaker.getX());
        SmartDashboard.putNumber("gety", pose.get().getY());
    }
    /*
     * List of working Limelight Distance | Shoulder Angle:
     * 3.0 | -0.5
     * 
     */
}