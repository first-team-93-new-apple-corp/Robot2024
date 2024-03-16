package frc.robot.subsystems.Vision;


import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.Vision.IO.VisionIO;
import frc.robot.subsystems.Vision.IO.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
    
    private final VisionIO m_io;
    private SwerveDriveSubsystem m_DriveSubsystem;
    private VisionIOInputs m_Inputs = new VisionIOInputs();

    public VisionSubsystem(VisionIO io, SwerveDriveSubsystem DriveSubsystem) {
        m_io = io;
        m_DriveSubsystem = DriveSubsystem;
        initialize();
    }

    

    protected void initialize() {
        m_io.updateValues(m_Inputs, m_DriveSubsystem, m_DriveSubsystem.m_SwerveDrivePoseEstimator.getEstimatedPosition());
    }

    public Pose2d getPose() {
        m_io.updateValues(m_Inputs, m_DriveSubsystem, m_DriveSubsystem.m_SwerveDrivePoseEstimator.getEstimatedPosition());
        return m_io.getPose(m_Inputs.pose);
    }
    
    public double getLatency(){
        m_io.updateValues(m_Inputs, m_DriveSubsystem, m_DriveSubsystem.m_SwerveDrivePoseEstimator.getEstimatedPosition());
        return m_io.getLatency(m_Inputs.tl, m_Inputs.cl, m_Inputs.latency);
    }
    public Boolean hasTargets() {
        m_io.updateValues(m_Inputs, m_DriveSubsystem, m_DriveSubsystem.m_SwerveDrivePoseEstimator.getEstimatedPosition());
        return m_io.hasTargets(m_Inputs.tv);
    }

    @Override
    public void periodic() {
        m_io.updateValues(m_Inputs, m_DriveSubsystem, m_DriveSubsystem.m_SwerveDrivePoseEstimator.getEstimatedPosition());
        SignalLogger.writeDoubleArray("Vision: Limelight Data", new double[]{m_Inputs.tx,m_Inputs.ty, m_Inputs.ta, m_Inputs.tl, m_Inputs.cl});
        SignalLogger.writeDouble("Vision:ID of tag in view", m_Inputs.tid);
        SignalLogger.writeDoubleArray("Vision:Pose of the Bot (WpiBlue)", m_Inputs.botpose_wpiblue);
        SignalLogger.writeDoubleArray("Vision:TargetPose Robot Space", m_Inputs.targetpose_robotspace);
        
        SmartDashboard.putBoolean("Has targets", hasTargets());
    }
}