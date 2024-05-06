// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Preflight;
import frc.robot.commands.ShooterCommand;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.*;
import com.pathplanner.lib.commands.PathfindingCommand;

public class Robot extends TimedRobot {
  static Joystick m_Joystick1 = new Joystick(0);
  static Joystick m_Joystick2 = new Joystick(1);
  static XboxController op = new XboxController(2);

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private LEDSubsystem m_LED;
  Constants constants;
  private ElevatorCommand m_Elevator;
  private ClimberCommand m_Climber;
  public Preflight m_Preflight;
  public IntakeCommand m_IntakeCommand;
  private SwerveDriveSubsystem m_SwerveDriveSubsystem;
  private ShooterCommand m_Shooter;
  // DigitalOutput test = new DigitalOutput(3);
  public Pigeon2 getPigeon() {
    return m_robotContainer.getPigeon();
  }

  @Override
  public void robotInit() {
    if (Utils.isSimulation()) {
      constants = new Constants("SIM");
    } else {
      constants = new Constants("2024");
    }
    m_robotContainer = new RobotContainer(constants, m_Joystick1, m_Joystick2, op);
    m_IntakeCommand = m_robotContainer.m_IntakeCommand;
    m_Climber = m_robotContainer.m_ClimberCommand;
    m_Elevator = m_robotContainer.m_ElevatorCommand;
    m_Preflight = m_robotContainer.m_PreflightCommand;
    m_LED = m_robotContainer.m_LedSubsystem;
    m_Shooter = m_robotContainer.m_ShooterCommand;
    m_SwerveDriveSubsystem = m_robotContainer.getDrive();
    m_Elevator.initOnce();
    // m_SwerveDriveSubsystem.configAuto();
    // m_UsbCameraSubsystem.register();
    SmartDashboard.putBoolean("Preflight Done?", false);
    m_LED.startup();
    PathfindingCommand.warmupCommand();
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
    m_robotContainer.updateVision();
    // m_robotContainer.pathTrajectory();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    // m_Elevator.disable();
    m_SwerveDriveSubsystem.m_SwerveDrivePoseEstimator.resetPosition(new Rotation2d(m_SwerveDriveSubsystem.getHeading()),
        m_SwerveDriveSubsystem.getModulePositions(),
        m_SwerveDriveSubsystem.getPose());
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // m_SwerveDriveSubsystem.m_SwerveDrivePoseEstimator.update(new Rotation2d(Math.toRadians(m_SwerveDriveSubsystem.getHeading())), m_SwerveDriveSubsystem.getModulePositions());
    // m_robotContainer.m_Field2d.setRobotPose(m_SwerveDriveSubsystem.m_SwerveDrivePoseEstimator.getEstimatedPosition());
    // m_Elevator.disable();
    m_robotContainer.updateVision();
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.m_Field2d.getObject("Traj").setTrajectory(new Trajectory());
  }

  @Override
  public void teleopInit() {
    // THIS SHOULDN'T BE RAN PERIODIC!!!!!!!!!!!!!
    m_robotContainer.configureBindings();
    // ^^^
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // m_IntakeCommand.schedule();
    // m_Shooter.schedule();
    m_Elevator.schedule();
    // m_Climber.schedule();
    m_robotContainer.updateValues();
    m_robotContainer.m_loop.poll();
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_Preflight.resetPreflight();
    m_Preflight.schedule();
    // m_robotContainer.m_ElevatorSubsystem.zero();
  }

  @Override
  public void testPeriodic() {
    m_robotContainer.m_ElevatorSubsystem.runElevator();
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
    m_SwerveDriveSubsystem.updateSimState(0.020, RobotController.getBatteryVoltage());
  }
}
