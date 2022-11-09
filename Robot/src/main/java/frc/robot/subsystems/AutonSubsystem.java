package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
/*----------------------------------------------------------------------------*/
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.StopDriveCommand;
import java.util.List;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;

public class AutonSubsystem extends SubsystemBase {
  PhotonCamera camera;
  PhotonCamera left;
  PhotonCamera right;
  TrajectoryConfig OnTheFlyConfig;
  Transform3d transform;

  public Command getTrajectoryCommand(
    DriveSubsystem m_DriveSubsystem,
    String PathName,
    boolean isFirstPath,
    double maxVel,
    double maxAccel
  ) {
    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.

  PathPlannerTrajectory trajectory = PathPlanner.loadPath(PathName, maxVel, maxAccel); 
    return new SequentialCommandGroup(
      new InstantCommand(
        () -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            m_DriveSubsystem.resetOdometry(
              trajectory.getInitialHolonomicPose()
            );
          }
        }
      ),
      new PPSwerveControllerCommand(
        trajectory,
        m_DriveSubsystem::getPose, // Pose supplier
        m_DriveSubsystem.Kinematics, // SwerveDriveKinematics
        new PIDController(1, 0, 0), // if overtuned robot will never stop
        new PIDController(1, 0, 0), // if overtuned, robot will never stop
        new PIDController(1, 0, 0), // if undertuned, robot will not reach chassis heading
        m_DriveSubsystem::setModuleStates, // Module states consumer
        m_DriveSubsystem // Requires this drive subsystem
      ),
      new StopDriveCommand(m_DriveSubsystem)
    );
  }

  @Override
  public void periodic() {}
}
