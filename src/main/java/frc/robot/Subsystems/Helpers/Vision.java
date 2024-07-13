package frc.robot.subsystems.Helpers;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Vision extends SubsystemBase {
    private NetworkTableEntry limelight = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose_targetspace");
    
    // private SwerveDriveSubsystem m_Drive;
    private Double[] values;
    private Double[] defaultValues;
    private double rotation;
    private PIDController pid = new PIDController(0.2, 0, 0.05);
    // private SlewRateLimiter slew = new SlewRateLimiter(0.4);
    // private SwerveRequest.RobotCentric drivereq = new SwerveRequest.RobotCentric();
    public Vision() {
        defaultValues = new Double[6];
        defaultValues[0] = 0.;
        defaultValues[1] = 0.;
        defaultValues[2] = 0.;
        defaultValues[3] = 0.;
        defaultValues[4] = 0.;
        defaultValues[5] = 0.;
        // this.m_Drive = m_Drive;
    }
    public double pointToCalc() {
        return-pid.calculate(rotation, 0);
    }
    @Override
    public void periodic() {
        values = limelight.getDoubleArray(defaultValues);
        rotation = values[4];
        SmartDashboard.putNumberArray("limelight values", values);
        SmartDashboard.putNumber("rotation", rotation);
    }
}
