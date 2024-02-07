// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Preflight;

public class Robot extends TimedRobot {
  XboxController op = new XboxController(2);
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private ShooterCommand m_Shooter = new ShooterCommand();
  private IntakeCommand m_Intake = new IntakeCommand();
  private ElevatorCommand m_Elevator = new ElevatorCommand(op);
  private ClimberCommand m_Climber = new ClimberCommand(op);
  private Preflight m_Preflight = new Preflight();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_Elevator.initOnce();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    m_Intake.schedule();
    m_Shooter.schedule();
    m_Elevator.schedule();
    m_Climber.schedule();
    m_robotContainer.configAuto();
    m_robotContainer.updateValues();
    m_robotContainer.configureBindings();
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_Preflight.schedule();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
