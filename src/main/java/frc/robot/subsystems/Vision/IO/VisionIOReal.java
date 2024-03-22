// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Vision.IO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public class VisionIOReal implements VisionIO {
    private final NetworkTable m_limelight;
    Pose2d pose;
    double tv;

  public VisionIOReal(VisionConstants constants) {
    m_limelight = NetworkTableInstance.getDefault().getTable(constants.CameraName);
    pose = new Pose2d();

  }


  /**
   * Updates the set of loggable inputs.
   */
    public void updateValues(VisionIOInputs inputs, SwerveDriveSubsystem m_DriveSubsystem, Pose2d lastpose) {
        inputs.tx = m_limelight.getEntry("tx").getDouble(0);
        inputs.ty = m_limelight.getEntry("ty").getDouble(0);
        inputs.tv = m_limelight.getEntry("tv").getDouble(0);
        tv = m_limelight.getEntry("tv").getDouble(0);
        inputs.ta = m_limelight.getEntry("ta").getDouble(0);
        inputs.tid = m_limelight.getEntry("tid").getDouble(0);
        inputs.targetpose_robotspace = m_limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        inputs.botpose_wpiblue = m_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        pose = new Pose2d(inputs.botpose_wpiblue[0], inputs.botpose_wpiblue[1], new Rotation2d(Math.toRadians(inputs.botpose_wpiblue[5])));
    }

    /** 
     * Turns the Limelight on
     */
    public void LimeLightOn() {
        m_limelight.getEntry("ledMode").setNumber(3);
    }
    
    /** 
     * Turns the Limelight off
     */
    public void LimeLightOff() {
        m_limelight.getEntry("ledMode").setNumber(0);
    }
    @Override
    public double getLatency(double tl, double cl, double latency) {
        return (tl + cl)/1000;
    }

}
