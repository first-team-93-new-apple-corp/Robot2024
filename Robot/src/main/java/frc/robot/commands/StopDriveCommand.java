// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.CustomRotationHelper;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class StopDriveCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private DriveSubsystem m_DriveSubsystem;

  private double x = 0;
  private double y = 0;
  private double z = 0;


  public StopDriveCommand(
      DriveSubsystem m_DriveSubsystem) {
    this.m_DriveSubsystem = m_DriveSubsystem;

    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("EXITED AUTON PATH");
    // m_DriveSubsystem.drive(0, 0, 0, true, DriveConstants.Center);
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
