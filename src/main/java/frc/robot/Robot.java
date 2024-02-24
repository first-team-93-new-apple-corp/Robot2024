// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.VisionCommand;
// import frc.robot.commands.ShooterCommand;
// import frc.robot.commands.ClimberCommand;
// import frc.robot.commands.ElevatorCommand;
// import frc.robot.commands.IntakeCommand;
// import frc.robot.subsystems.CameraSubsystem;

public class Robot extends TimedRobot {
  // CameraSubsystem m_cam = new CameraSubsystem();

  VisionCommand m_Vision;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private LEDCommand m_LED;
  // private ShooterCommand m_Shooter = new ShooterCommand();
  // private IntakeCommand m_Intake = new IntakeCommand();
  // private ElevatorCommand m_Elevator = new ElevatorCommand(op);
  // private ClimberCommand m_Climber = new ClimberCommand(op);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_Vision = new VisionCommand(m_robotContainer.getDrivetrain());
    m_LED = new LEDCommand();
    // m_cam.register();
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
    // m_Intake.schedule();
    // m_Shooter.schedule();
    // m_Elevator.schedule();
    // m_Climber.schedule();
      // m_Camera.schedule();
    m_LED.schedule();
    m_Vision.schedule();
    m_robotContainer.updateValues();
    m_robotContainer.configureBindings();
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
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
