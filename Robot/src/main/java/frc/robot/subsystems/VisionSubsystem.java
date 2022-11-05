package frc.robot.subsystems;

/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera;
  PhotonCamera left;
  PhotonCamera right;
  TrajectoryConfig OnTheFlyConfig;
  Transform3d transform;

  public VisionSubsystem() {
    camera = new PhotonCamera("2K_WEBCAM");
    left = new PhotonCamera("USB CAMERA LEFT");
    right = new PhotonCamera("USB CAMERA RIGHT");
    if (true) { // TODO change
      camera.setPipelineIndex(0);
      left.setPipelineIndex(0);
      right.setPipelineIndex(0);
    } else {
      camera.setPipelineIndex(0);
      left.setPipelineIndex(0);
      right.setPipelineIndex(0);
    }

    SetDriverMode(true);
  }

  public void SetDriverMode(boolean on) {
    camera.setDriverMode(on);
  }

  PhotonPipelineResult latestResult;

  public Pose2d getCameraPose2d() {
    SetDriverMode(false);
    latestResult = camera.getLatestResult();
    SetDriverMode(true);

    if (latestResult.hasTargets()) {
      Transform3d Result = latestResult.getBestTarget().getBestCameraToTarget();

      //System.out.println();
      return new Pose2d(
        new Translation2d(
        Result.getX(),
        Result.getY()
        ),
        new Rotation2d() //TODO may need to return the correct rotation 2d
      );
    }

    return null;
  }

  //put this code in when we are ready for multicam

  // public Pose2d getPose2dLeft() {
  //   latestResult = left.getLatestResult();
  //   if (latestResult.hasTargets()) {
  //     //System.out.println();
  //     return new Pose2d(latestResult.getBestTarget().getCameraToTarget().getTranslation(),
  //         new Rotation2d(latestResult.getBestTarget().getCameraToTarget().getX(),
  //             latestResult.getBestTarget().getCameraToTarget().getY()).plus(Rotation2d.fromDegrees(45)));
  //   }

  //   return null;

  // }

  // public Pose2d getPose2dRight() {
  //   latestResult = right.getLatestResult();
  //   if (latestResult.hasTargets()) {
  //     //System.out.println();
  //     return new Pose2d(latestResult.getBestTarget().getCameraToTarget().getTranslation(),
  //         new Rotation2d(latestResult.getBestTarget().getCameraToTarget().getX(),
  //             latestResult.getBestTarget().getCameraToTarget().getY()).minus(Rotation2d.fromDegrees(45)));

  // }
  // return null;
  // }
  // BEGINING OF MULTI CAM TESTING

  // public Pose2d MasterGetPose2d2(){
  //   double frontTime = 999999999;
  //   double leftTime = 999999999;
  //   double rightTime = 999999999;
  //   if(camera.getLatestResult().hasTargets()){
  // frontTime =  getTrajectoryNotMovingWithDifCamera(getPose2d()).getTotalTimeSeconds();
  //   } else if (left.getLatestResult().hasTargets()){
  //    leftTime =  getTrajectoryNotMovingWithDifCamera(getPose2dLeft()).getTotalTimeSeconds();
  //   } else if (right.getLatestResult().hasTargets()){
  //      rightTime =  getTrajectoryNotMovingWithDifCamera(getPose2dRight()).getTotalTimeSeconds();
  //   }
  //   if(frontTime < leftTime && frontTime < rightTime){
  //     return getPose2d();
  //   } else if (leftTime < frontTime && leftTime < rightTime){
  //     return getPose2dLeft();
  //   } else {
  //     return getPose2dRight();
  //   }
  // }

  // public Trajectory getTrajectoryNotMovingWithDifCamera(Pose2d pose) {
  //   if (pose != null) {

  //     Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(), pose,
  //         FerbConfig.config);

  //     return traj;
  //   }
  //   return null;
  // }

  // public Pose2d MasterGetPose2d() {

  //   //this one just gets how far the ball is off of center from the bot. ball could be 20 ft away on left but on the right side of the frame while theres one on the right 2 feet away in the center of frame and it will go to the ball in the left cam
  //   if (camera.getLatestResult().hasTargets()) {
  //     return new Pose2d(camera.getLatestResult().getBestTarget().getCameraToTarget().getTranslation(),
  //         new Rotation2d(camera.getLatestResult().getBestTarget().getCameraToTarget().getX(),
  //             camera.getLatestResult().getBestTarget().getCameraToTarget().getY()));
  //   } else if (left.getLatestResult().hasTargets() && right.getLatestResult().hasTargets()) {
  //     if (left.getLatestResult().getBestTarget().getCameraToTarget()
  //         .getX() > -(right.getLatestResult().getBestTarget().getCameraToTarget().getX())) {
  //       return new Pose2d(left.getLatestResult().getBestTarget().getCameraToTarget().getTranslation(),
  //           new Rotation2d(left.getLatestResult().getBestTarget().getCameraToTarget().getX(),
  //               left.getLatestResult().getBestTarget().getCameraToTarget().getY()).minus(Rotation2d.fromDegrees(45)));
  //     } else {
  //       return new Pose2d(right.getLatestResult().getBestTarget().getCameraToTarget().getTranslation(),
  //           new Rotation2d(right.getLatestResult().getBestTarget().getCameraToTarget().getX(),
  //               right.getLatestResult().getBestTarget().getCameraToTarget().getY()).plus(Rotation2d.fromDegrees(45)));
  //     }

  //   } else if (left.getLatestResult().hasTargets()) {
  //     return new Pose2d(left.getLatestResult().getBestTarget().getCameraToTarget().getTranslation(),
  //         new Rotation2d(left.getLatestResult().getBestTarget().getCameraToTarget().getX(),
  //             left.getLatestResult().getBestTarget().getCameraToTarget().getY()).plus(Rotation2d.fromDegrees(45)));
  //   } else if (right.getLatestResult().hasTargets()) {
  //     return new Pose2d(right.getLatestResult().getBestTarget().getCameraToTarget().getTranslation(),
  //         new Rotation2d(right.getLatestResult().getBestTarget().getCameraToTarget().getX(),
  //             right.getLatestResult().getBestTarget().getCameraToTarget().getY()).minus(Rotation2d.fromDegrees(45)));

  //   }
  //   return null;
  // }

  // public Pose2d TestCameraPose(PhotonCamera obj) {
  //   latestResult = obj.getLatestResult();
  //   if (latestResult.hasTargets()) {
  //     //System.out.println();
  //     return new Pose2d(latestResult.getBestTarget().getCameraToTarget().getTranslation(),
  //         new Rotation2d(latestResult.getBestTarget().getCameraToTarget().getX(),
  //             latestResult.getBestTarget().getCameraToTarget().getY()));
  //   }

  //   return null;
  // }

  // public Pose2d TestCameraPose2(PhotonCamera obj) {
  //   latestResult = obj.getLatestResult();
  //   if (latestResult.hasTargets()) {
  //     //System.out.println();
  //     return new Pose2d(latestResult.getBestTarget().getCameraToTarget().getTranslation(),
  //         new Rotation2d(latestResult.getBestTarget().getCameraToTarget().getX(),
  //             latestResult.getBestTarget().getCameraToTarget().getY()));
  //   }

  //   return null;
  // }

  // public boolean HasTarget2() {
  //   if (getSide() == 0 && !camera.getLatestResult().hasTargets()) {
  //     return false;
  //   }
  //   return true;
  // }

  // public PhotonCamera getSide2() {
  //   if (left.getLatestResult().hasTargets() && right.getLatestResult().hasTargets()) {
  //     if (left.getLatestResult().getBestTarget().getArea() > right.getLatestResult().getBestTarget().getArea()) {
  //       return left;
  //     } else {
  //       return right;
  //     }
  //   } else if (left.getLatestResult().hasTargets()) {
  //     return left;
  //   } else if (right.getLatestResult().hasTargets()) {
  //     return right;
  //   } else {
  //     return camera;
  //   }
  // }


  // public int getSide() {
  //   if (left.getLatestResult().hasTargets() && right.getLatestResult().hasTargets()) {
  //     if (left.getLatestResult().getBestTarget().getArea() > right.getLatestResult().getBestTarget().getArea()) {
  //       return 1;
  //     } else {
  //       return 2;
  //     }
  //   } else if (left.getLatestResult().hasTargets()) {
  //     return 1;
  //   } else if (right.getLatestResult().hasTargets()) {
  //     return 2;
  //   } else {
  //     return 0;
  //   }
  // }

  // END OF MULTI CAM TESTING
  public PathPlannerTrajectory getTrajectoryMoving(double currentVelocity) {
    OnTheFlyConfig =
      new TrajectoryConfig(
        DriveConstants.Max_Strafe_Speed,
        DriveConstants.Max_Strafe_Speed
      );

    OnTheFlyConfig.setStartVelocity(currentVelocity);
    OnTheFlyConfig.setEndVelocity(0);

    Pose2d pose = getCameraPose2d();
    if (pose != null) {
      PathPlannerTrajectory traj = PathPlanner.generatePath(
        null, 
        null,
        null
      ); // this will work for all trajectories after the first one but not for the first
      // becuase

      return traj;
    }
    return null;
  }

  public boolean isClose() {
    Pose2d pose = getCameraPose2d();
    if (pose != null) {
      if (
        pose.getTranslation().getX() < 0.5 && pose.getTranslation().getY() < 0.5
      ) {
        return true;
      }
    }
    return false;
  }

  public PathPlannerTrajectory getTrajectory(double startVelo, double endVelo, Pose2d PoseOdometry) {
    PathPlanner.generatePath(null, null, null, null);
    Pose2d pose = getCameraPose2d();
    if (pose != null) {
      PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(4, 3), //make Constants
        new PathPoint(null, null),
        new PathPoint(null, null)
      );

      return traj;
    }
    return null;
  }

  public boolean HasTarget() {
    return camera.getLatestResult().hasTargets();
  }

  @Override
  public void periodic() {
  }
}