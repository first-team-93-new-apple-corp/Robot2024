// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Vision.IO;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class VisionIOPhotonReal implements VisionIO {
    private Pose2d pose;
    private AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private Transform3d robotToCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    private String CameraName;
  public VisionIOPhotonReal(VisionConstants constants) {
    CameraName = constants.CameraName;
    camera = new PhotonCamera(CameraName);


    robotToCamera = new Transform3d(new Pose3d(), constants.RobotToCamera);
    // Add this camera to the vision system simulation with the given robot-to-camera transform.
    photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
    pose = new Pose2d(0,0, Rotation2d.fromDegrees(0));
  }


  
    /**
    * Updates our Vision values
    */
    @Override
    public void updateValues(VisionIOInputs inputs, SwerveDriveSubsystem m_DriveSubsystem, Pose2d lastpose) {
      result = camera.getLatestResult();
        if (result.hasTargets()) {
          target = result.getBestTarget();
          inputs.tx = target.getYaw();
          inputs.ty = target.getPitch();
          inputs.ta = target.getArea();
          inputs.tid = target.getFiducialId();
          inputs.latency = result.getLatencyMillis() / 1000.0;
          // pose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagLayout.getTagPose(target.getFiducialId()).get(), robotToCamera).toPose2d(); 
          // pose = photonPoseEstimator.update().get().estimatedPose.toPose2d();
          try {
            pose = photonPoseEstimator.update(result).get().estimatedPose.toPose2d();
          } catch (Exception e) {
            pose = lastpose;
          }
        }
    }

    @Override
    public Pose2d getPose(Pose2d pose) {
        if (result.hasTargets()) {
            return photonPoseEstimator.update(result).get().estimatedPose.toPose2d();
        } else {
            return this.pose;
        }
    }
    /** 
     * Updates the Camera sim
     * You must call this every Loop in Sim
     */


    @Override
    public boolean hasTargets(double tv) {
        return result.hasTargets();
    }
    @Override
    public double getLatency(double tl, double cl, double latency) {
      return latency;
    }
}
