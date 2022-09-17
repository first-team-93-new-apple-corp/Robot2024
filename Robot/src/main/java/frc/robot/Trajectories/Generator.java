package frc.robot.Trajectories;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;

public class Generator {
  static Trajectory trajectory = new Trajectory();

  public Generator() {}

  public static Trajectory getTrajectory(String TrajectoryJson) {
    String trajectoryJSON = "output/" + TrajectoryJson + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem
        .getDeployDirectory()
        .toPath()
        .resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (Exception e) {
      DriverStation.reportError(
        "Unable to open trajectory: " + trajectoryJSON,
        e.getStackTrace()
      );
    }

    return trajectory;
  }
}
