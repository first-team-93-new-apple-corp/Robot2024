// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_DriveSubsystem;

  public DriveCommand(DriveSubsystem m_DriveSubsystem) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() {
    // encoders should be absolute so no need to reset them except for once?
    // test encoder values
    // reset them right before matches?
    // will require testing to figure out

    // m_DriveSubsystem.resetEncoders();
    

  }

  @Override
  public void execute() {
    // m_DriveSubsystem.getEncoderValues();
    m_DriveSubsystem.drive(0.5, 0, 0, false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
