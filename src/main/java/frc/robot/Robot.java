// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
// import frc.robot.subsystems.USBCameraSubsystem;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.Preflight;
import com.ctre.phoenix6.hardware.*;
public class Robot extends TimedRobot {
  static Joystick m_Joystick1 = new Joystick(0);
  static Joystick m_Joystick2 = new Joystick(1);
  static XboxController op = new XboxController(2);

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  // private USBCameraSubsystem m_UsbCameraSubsystem = new USBCameraSubsystem();
  private ShooterCommand m_Shooter = new ShooterCommand();
  private ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private IntakeCommand m_Intake = new IntakeCommand(m_ShooterSubsystem);
  private ElevatorCommand m_Elevator = new ElevatorCommand(op);
  private ClimberCommand m_Climber = new ClimberCommand(op);
  private LEDCommand m_LED = new LEDCommand(op);
  private SwerveDriveSubsystem m_SwerveDriveSubsystem;
  private Preflight m_Preflight;
  public Pigeon2 getPigeon() {
    return m_robotContainer.getPigeon();
  }
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer(m_Joystick1, m_Joystick2, op);
    m_SwerveDriveSubsystem = m_robotContainer.getDrive();
    m_Preflight = new Preflight(op, m_Elevator, m_Intake);
    m_Elevator.initOnce();
    m_SwerveDriveSubsystem.configAuto();
    // m_UsbCameraSubsystem.register();
    SmartDashboard.putBoolean("Preflight Done?", false);

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
    m_LED.schedule();
    m_robotContainer.updateValues();
    m_robotContainer.configureBindings();
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_Preflight.resetPreflight();
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
