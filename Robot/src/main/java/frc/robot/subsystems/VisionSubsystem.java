package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    String limelightName;
    NetworkTable limelight;
    DriveSubsystem m_DriveSubsystem;
    double x, forward, size = 0;
    boolean fieldRel, recent = false;
    Translation2d COR = DriveConstants.dCenter;
    PIDController turnPID = new PIDController(0.0085, 0, 0);
    PIDController drivePID = new PIDController(0.5, 0, 0);
    double rotate;
    Timer m_Timer;

    public VisionSubsystem(String limelightName) {
        limelightName = this.limelightName;
        limelight = NetworkTableInstance.getDefault().getTable("limelight-front");
        m_DriveSubsystem = new DriveSubsystem();
        m_Timer = new Timer();
        m_Timer.reset();
    }

    public void updateValues() {
        NetworkTableEntry tx = limelight.getEntry("tx");
        NetworkTableEntry ta = limelight.getEntry("ta");

        // read values periodically
        x = tx.getDouble(0.0);
        size = ta.getDouble(0.0);

        // post to smart dashboard periodically
        SmartDashboard.putBoolean("has target", hasTargets());
        // System.out.println("updating values");
    }

    public boolean hasTargets() {
        // if no target found
        SmartDashboard.putBoolean("Recent", recent);
        SmartDashboard.putNumber("Timer", m_Timer.get());
        if (size == 0) {
            // if we haven't started a timer yet
            if (recent) {
                m_Timer.start();
                recent = false;
            }
            // if it has been less that a half second since we lost target return true
            if (m_Timer.get() < 0.25 && m_Timer.get() != 0.0) {
                return true;
            } else { // we lost target for more than a half second, stopping timer and returning false
                m_Timer.stop();
                m_Timer.reset();
                recent = false;
                return false;
            }
        } else {
            // Target found, stop timer and allow the timer to start again if the limelight loses target
            m_Timer.stop();
            m_Timer.reset();
            recent = true;
            return true;
        }
        
    }

    public void followTape() {
        // fov = 60 limelight returns -30 to 30
        if (hasTargets()) {
            rotate = turnPID.calculate(x, 0);
            forward = drivePID.calculate(size, 0.5);
            MathUtil.clamp(forward, -0.15, 0.5);
            m_DriveSubsystem.drive(forward, 0, rotate, false, DriveConstants.dCenter);
        } else {
            m_DriveSubsystem.drive(0, 0, 0, false, DriveConstants.dCenter);

        }

        // System.out.println("following tape");
    }

    @Override
    public void periodic() {
        updateValues();
        followTape();
    }
}
