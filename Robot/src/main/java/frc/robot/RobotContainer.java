// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.TelescopingSubsystem;

public class RobotContainer {
  TelescopingSubsystem m_telescopingSubsystem;
  XboxController m_F310; 

  JoystickButton arm_Button; 
  JoystickButton arm_In; 
  JoystickButton arm_Out; 

  // Joysticks

  // Other Definitions
  

  public RobotContainer() {

    //Controllers
    m_F310 = new XboxController(0); 
    arm_Button = new JoystickButton(m_F310, 1);
    arm_In = new JoystickButton(m_F310, 5);
    arm_Out = new JoystickButton(m_F310, 6);
    //Subsystems
    m_telescopingSubsystem = new TelescopingSubsystem();

    //Commands

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //modifying to be handled by the command while we tune the mechanisms... 

    // arm_Button.whileTrue(new TestingArmCommand(m_telescopingSubsystem));
    // arm_In.whileTrue(new ManualTelescopeCommand(m_telescopingSubsystem, -0.1));
    // arm_Out.whileTrue(new ManualTelescopeCommand(m_telescopingSubsystem, 0.1));
    
  }

  public Command getTeleopCommand() {
    return null;
  }

  public Command getAutonomousCommand() {
     return null;
  }
}
