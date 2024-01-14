package frc.robot.subsystems;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonSubsystem extends SubsystemBase {

  TrajectoryConfig OnTheFlyConfig;
  Transform3d transform;

  public Command getTrajectoryCommand(
    DriveSubsystem m_DriveSubsystem,
    String PathName,
    boolean isFirstPath,
    double maxVel,
    double maxAccel
  ) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(
      PathName,
      maxVel,
      maxAccel
    );
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if (isFirstPath) {
          m_DriveSubsystem.resetOdometry(trajectory.getInitialPose());
        }
      }),
      new PPSwerveControllerCommand(
        trajectory,
        m_DriveSubsystem::getPose, // Pose supplier
        m_DriveSubsystem.Kinematics, // SwerveDriveKinematics
        new PIDController(5, 0, 0), // if over tuned robot will never stop
        new PIDController(5, 0, 0), // if over tuned, robot will never stop
        new PIDController(5, 0, 0), // if under tuned, robot will not reach chassis heading
        m_DriveSubsystem::setModuleStates, // Module states consumer
        m_DriveSubsystem // Requires this drive subsystem
      )
    );
  }

  @Override
  public void periodic() {}
}
