package frc.robot.subsystems.Helpers;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private NetworkTableEntry limelight = NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose_targetspace");
    private Pose2d pose;
    Pigeon2 pigeon2;
    // private SwerveDriveSubsystem m_Drive;
    private Double[] values;
    private Double[] defaultValues;
    private double rotation;
    private PIDController pid = new PIDController(0.05, 0, 0);
    // private SlewRateLimiter slew = new SlewRateLimiter(0.4);
    // private SwerveRequest.RobotCentric drivereq = new SwerveRequest.RobotCentric();
    public Vision(Pigeon2 pigeon2) {
        this.pigeon2= pigeon2;
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
        // LimelightHelpers.SetRobotOrientation("limelight-front", pigeon2.getYaw().getValueAsDouble(), pigeon2.getRate(), pigeon2.getPitch().getValueAsDouble(), 0, pigeon2.getRoll().getValueAsDouble(), 0);
        // pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front").pose;
        values = limelight.getDoubleArray(defaultValues);
        
        rotation = values[4];
        SmartDashboard.putNumberArray("limelight values", values);
        SmartDashboard.putNumber("rotation", rotation);
    }
}
