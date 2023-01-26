// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TelescopingSubsystem;

public class RobotContainer {
  TelescopingSubsystem m_telescopingSubsystem;

  // Joysticks

  // Other Definitions
  

  public RobotContainer() {

    //Controllers

    //Subsystems
    m_telescopingSubsystem = new TelescopingSubsystem();
    //Commands

    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  public Command getTeleopCommand() {
    return null;
  }

  public Command getAutonomousCommand() {
     return null;
  }
}
