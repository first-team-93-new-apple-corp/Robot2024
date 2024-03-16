// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Vision.IO;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class VisionIOSim implements VisionIO {
    private final NetworkTable m_limelight;
    Pose2d pose;
    private VisionSystemSim visionSim = new VisionSystemSim("main");
    private AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    private SimCameraProperties cameraProp = new SimCameraProperties();
    private PhotonCamera camera = new PhotonCamera("cameraName");
    private PhotonCameraSim cameraSim;
    PhotonPipelineResult result;
    PhotonTrackedTarget target;
    Transform3d robotToCamera;
    Rotation3d robotToCameraRot;
    Translation3d robotToCameraTrans;
    PhotonPoseEstimator photonPoseEstimator;
  public VisionIOSim() {
    visionSim.addAprilTags(tagLayout);

    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProp);
    
    // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
    // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
    robotToCameraTrans = new Translation3d(0.1, 0, 0.5);
    // and pitched 15 degrees up.
    robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
    robotToCamera = new Transform3d(robotToCameraTrans, robotToCameraRot);

    // Add this camera to the vision system simulation with the given robot-to-camera transform.
    visionSim.addCamera(cameraSim, robotToCamera);
    photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    pose = new Pose2d(0,0, Rotation2d.fromDegrees(0));
  }


  
    /**
    * Updates our Vision values
    */
    @Override
    public void updateValues(VisionIOInputs inputs, SwerveDriveSubsystem m_DriveSubsystem, Pose2d lastpose) {
      visionSim.update(m_DriveSubsystem.m_SwerveDrivePoseEstimator.getEstimatedPosition());
      result = camera.getLatestResult();
        if (result.hasTargets()) {
          target = result.getBestTarget();
          inputs.tx = target.getYaw();
          inputs.ty = target.getPitch();
          inputs.ta = target.getArea();
          inputs.tid = target.getFiducialId();
          inputs.targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
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
      // return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagLayout.getTagPose(target.getFiducialId()).get(), robotToCamera).toPose2d();
      // if (photonPoseEstimator.update().isPresent()) {
        return pose;
      // } else {
      //   return new Pose2d();
      // }
    }
    /** 
     * Updates the Camera sim
     * You must call this every Loop in Sim
     */
    public void updateVisionSim(SwerveDriveSubsystem m_DriveSubsystem) {
        // Update with the simulated drivetrain pose. This should be called every loop in simulation.
        visionSim.update(m_DriveSubsystem.m_SwerveDrivePoseEstimator.getEstimatedPosition());
    }

    @Override
    public boolean hasTargets(double tv) {
        return result.hasTargets();
    }
    @Override
    public double getLatency(double tl, double cl, double latency) {
      return latency;
    }
}
