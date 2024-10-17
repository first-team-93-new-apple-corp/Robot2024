// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.TankDriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Platforms.PlatformSubsystem;

public class RobotContainer {
  private CommandXboxController m_XboxController = new CommandXboxController(0);

  private TankDriveSubsystem m_drivetrain = new TankDriveSubsystem();
  private ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private PlatformSubsystem m_platforms = new PlatformSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
        m_drivetrain.run(
            () -> m_drivetrain.drive(m_XboxController.getLeftY(), m_XboxController.getRightX())));
    
    m_XboxController.povDown().whileTrue(m_elevator.run(() -> m_elevator.moveDown()));
    m_XboxController.povUp().whileTrue(m_elevator.run(() -> m_elevator.moveUp()));
    m_XboxController.povCenter().whileTrue(m_elevator.run(() -> m_elevator.stop()));
    
    m_XboxController.b().whileTrue(m_platforms.run(() -> m_platforms.windUpRight()));
    m_XboxController.x().whileTrue(m_platforms.run(() -> m_platforms.windDownRight()));
    m_XboxController.povLeft().whileTrue(m_platforms.run(() -> m_platforms.windDownLeft()));
    m_XboxController.povRight().whileTrue(m_platforms.run(() -> m_platforms.windUpLeft()));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
