package frc.robot.subsystems.Vision.IO;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * arm subsystem hardware interface.
 */
public interface VisionIO {

  /**
   * Contains all of the input data received from hardware.
   */
  public static class VisionIOInputs {
    public double tx, ty;
    public double tl;
    public double ta;
    public double tid;
    public double cl;
    public double[] targetpose_robotspace, botpose, botpose_wpiblue;
    public double tv;
    public Pose2d pose;
    public double latency;
    }

    /**
     * Updates the set vision data.
     */
    public void updateValues(VisionIOInputs inputs, SwerveDriveSubsystem m_DriveSubsystem, Pose2d lastpose);
    
    public default Pose2d getPose(Pose2d pose) {
      return pose;
    }
    public default double getLatency(double tl, double cl, double latency) {
      return 0;
    }
    public default boolean hasTargets(double tv) {
      if (tv == 1) {
          return true;
      } else
          return false;
    }
}
