// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class StopDriveCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private DriveSubsystem m_DriveSubsystem;

  /**
   * Stops the Swerve Drive motors in their current position
   *
   * @end Ends Immediately
   * @param m_DriveSubsystem Drive Subsystem
   * 
   */
  public StopDriveCommand(
      DriveSubsystem m_DriveSubsystem) {
    this.m_DriveSubsystem = m_DriveSubsystem;

    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() {
    m_DriveSubsystem.stopMotors();
  }

  @Override
  public void execute() {
    }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
