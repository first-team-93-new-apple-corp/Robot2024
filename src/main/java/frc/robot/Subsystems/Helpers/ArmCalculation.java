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
import frc.robot.subsystems.ShooterSubsystem;
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
    double[] points;
    double speed;
    double test;

    // deltaX^2 + deltaY^2 + deltaZ^2 = Distance^2!
    private double setpoint;
    ShoulderSubsystem m_ShoulderSubsystem;
    ShooterSubsystem m_ShooterSubsystem;
    public Supplier<Pose2d> pose;

    public ArmCalculation(ShoulderSubsystem m_ShoulderSubsystem2, ShooterSubsystem m_ShooterSubsystem2,
            Supplier<Pose2d> pose) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.m_ShoulderSubsystem = m_ShoulderSubsystem2;
        this.m_ShooterSubsystem = m_ShooterSubsystem2;
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
        // *(pose.get().getX() - Constants.goals.RedSpeaker.getX()))
        // +(((pose.get().getY() - Constants.goals.RedSpeaker.getY()))
        // *((pose.get().getY() - Constants.goals.RedSpeaker.getY())))
        // +(((0 - Constants.goals.RedSpeaker.getZ()))
        // *((0 - Constants.goals.RedSpeaker.getZ()))));
        // b = (pose.get().getX() - Constants.goals.RedSpeaker.getX());
        // a = (Constants.goals.RedSpeaker.getZ());
        // c = ((a*a)+(b*b));
        // // d = ((a*a)+(b*b)+(c*c))
        // distanceToTarget = Math.sqrt(c);

        // setpoint = Math.acos(b/(distanceToTarget));
        // angleRad = setpoint;
        // setpoint = Math.toDegrees(angleRad)+45-20-12;
        // unclamped = (setpoint);

        // m_ShooterSubsystem.set(speed);
        test = findClossest((pose.get().getX()), poseXPoints);
        if (test == 15.0) {
            setpoint = -20;
            speed = 0.5;
        } else if (test == 14.5) {
            setpoint = -3;
            speed = 0.5;
        } else if (test == 14.25) {
            setpoint = 0;
            speed = 0.6;
        } else if (test == 14.0) {
            setpoint = 3;
            speed = 0.75;
        } else {
            setpoint = 10;
            speed = 0;
        }
        m_ShooterSubsystem.set(speed);
        setpoint = MathUtil.clamp(setpoint, -10, 30);
        m_ShoulderSubsystem.toSetpoint(setpoint);
    }

    public double findClossest(double input, double[] array) {
        double clossestValue = array[0];
        double smallestDifference = Math.abs(input - clossestValue);

        for (double value : array) {
            double difference = Math.abs(input - value);
            if (difference < smallestDifference) {
                smallestDifference = difference;
                clossestValue = value;

            }
        }
        return clossestValue;
    }

    double[] poseXPoints = { 15, 14.5, 14.25, 14, 13.5 };

    /*
     * PoseX, Angle, Speed(optional)
     * 1 (15, -20, 0.5)
     * 2 (14.5, -3, 0.5)
     * 3 (14, )
     * 4
     * 5
     */
    public Command calculate() {
        return this.runOnce(() -> calculateSetpoint());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distancetotarget", distanceToTarget);
        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putNumber("unclamped Setpoint", unclamped);
        SmartDashboard.putNumber("getx", pose.get().getX());
        SmartDashboard.putNumber("gety", pose.get().getY());
        SmartDashboard.putNumber("test setpoing", test);
    }
    /*
     * List of working Limelight Distance | Shoulder Angle:
     * 3.0 | -0.5
     * 
     */
}