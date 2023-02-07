// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.HumanDrive;
import frc.robot.commands.AutonCommands.CableBumpBlue1Pickup;
import frc.robot.commands.AutonCommands.DriveAndLevel;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  // Joysticks
  public Joystick Driver1;
  public Joystick Driver2;
  public XboxController F310;

  // Subsystem Definitions
  DriveSubsystem m_DriveSubsystem;
  AutonSubsystem m_AutonSubsystem; 

  // Command Definitions 
  HumanDrive m_DriveCommand; 
  

  // Other Definitions
  SendableChooser<Command> AutonChooser;
  

  public RobotContainer() {

    //Controllers
    Driver1 = new Joystick(0);
    Driver2 = new Joystick(1);
    F310 = new XboxController(2);

    //Subsystems
    m_DriveSubsystem = new DriveSubsystem(); 
    m_AutonSubsystem = new AutonSubsystem();

    //Commands
    m_DriveCommand = new HumanDrive(m_DriveSubsystem, Driver1, Driver2, F310); 
    

    // Auton Path Chooser
    AutonChooser = new SendableChooser<Command>();

    // AutonChooser.setDefaultOption("No Path", null);
    AutonChooser.addOption("Test Path", DriveAndLevel.generatePath(m_AutonSubsystem, m_DriveSubsystem));
    AutonChooser.setDefaultOption("CableBumpBlue1Pickup", CableBumpBlue1Pickup.generatePath(m_AutonSubsystem, m_DriveSubsystem));

    SmartDashboard.putData("Auton Chooser", AutonChooser);
  

    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  public Command getTeleopCommand() {
    return m_DriveCommand;
  }

  public Command getAutonomousCommand() {
     return AutonChooser.getSelected();
  }
}
