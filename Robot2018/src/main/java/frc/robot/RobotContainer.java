// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.TankDriveSubsystem;

public class RobotContainer {
  private CommandXboxController m_XboxController = new CommandXboxController(0);

  private TankDriveSubsystem m_drivetrain = new TankDriveSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
        m_drivetrain.run(
            () -> m_drivetrain.drive(m_XboxController.getLeftY(), m_XboxController.getRightX())));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
