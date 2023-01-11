// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;


public class ArmCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
 public ShoulderSubsystem m_ShoulderSubsystem;
 public TelescopingSubsystem m_TelescopingSubsystem;

  public ArmCommand(
      TelescopingSubsystem m_TelescopingSubsystem, ShoulderSubsystem m_ShoulderSubsystem) {
        this.m_ShoulderSubsystem = m_ShoulderSubsystem;
        this.m_TelescopingSubsystem = m_TelescopingSubsystem;
    addRequirements(m_ShoulderSubsystem,m_TelescopingSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_ShoulderSubsystem.RunShoulderMotors();
    }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
