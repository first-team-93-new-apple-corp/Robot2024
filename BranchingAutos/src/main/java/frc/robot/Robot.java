// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LED;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  // private final LED m_LED;

  public Robot() {
    m_robotContainer = new RobotContainer();
    // m_LED = new LED();
    
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
    addPeriodic(() -> {
      // m_robotContainer.m_LED.applyColorCycle(4, Color.kBlue, Color.kBlack);
      // m_robotContainer.m_LED.applyColorCycle(10, Color.kGreen, Color.kBlack, 2);
    }, 0.075, 0.005);
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
    // m_robotContainer.getIdleLEDs().schedule();
    m_robotContainer.m_LED.applyColorCycle(4, Color.kBlue, Color.kBlack);
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
