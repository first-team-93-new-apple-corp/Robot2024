// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  // Joysticks
  public Joystick Driver1;
  public Joystick Driver2;
  public XboxController Operator;


  //Subsystem Definitions

  //Command Definitions 

  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //Controllers
    Driver1 = new Joystick(0);
    Driver2 = new Joystick(1);
    Operator = new XboxController(2);

    //Subsystems

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
