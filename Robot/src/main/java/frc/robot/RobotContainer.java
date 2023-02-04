// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.OperatorSelectorCommand;
import frc.robot.commands.Grabber_Commands.Tuning_GrabberCommand;
import frc.robot.commands.Shoulder_Commands.Manual_ShoulderCommand;
import frc.robot.commands.Shoulder_Commands.Tuning_ShoulderCommand;
import frc.robot.commands.Telescope_Commands.Manual_TelescopeCommand;
import frc.robot.commands.Telescope_Commands.Tuning_TelescopeCommand;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.OperatorInterfaceSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class RobotContainer {

  // Subsystems
  TelescopingSubsystem m_telescopingSubsystem;
  GrabberSubsystem m_grabberSubsystem;
  ShoulderSubsystem m_ShoulderSubsystem;
  OperatorInterfaceSubsystem m_OperatorInterfaceSubsystem;

  // Commands
  Tuning_TelescopeCommand m_TelescopingCommand;
  // Tuning_GrabberCommand m_GrabberCommand;
  Tuning_ShoulderCommand m_ShoulderCommand;
  OperatorSelectorCommand m_OperatorSelectorCommand;

  // Controllers
  XboxController m_F310;

  // Buttons
  JoystickButton grabber_in_Button;
  JoystickButton grabber_out_Button;
  JoystickButton arm_Button;
  JoystickButton arm_In;
  JoystickButton arm_Out;

  // Other Definitions

  public RobotContainer() {

    // Controllers
    m_F310 = new XboxController(Constants.Joystick_Port.F310Port);
    grabber_in_Button = new JoystickButton(m_F310, Constants.F310.A);
    grabber_out_Button = new JoystickButton(m_F310, Constants.F310.B);
    arm_Button = new JoystickButton(m_F310, Constants.F310.Start);
    arm_In = new JoystickButton(m_F310, Constants.F310.X);
    arm_Out = new JoystickButton(m_F310, Constants.F310.Y);

    // Subsystems
    m_telescopingSubsystem = new TelescopingSubsystem();
    m_grabberSubsystem = new GrabberSubsystem();
    m_OperatorInterfaceSubsystem = new OperatorInterfaceSubsystem();
    m_ShoulderSubsystem = new ShoulderSubsystem();

    // Commands
    // m_TelescopingCommand = new Tuning_TelescopeCommand(m_telescopingSubsystem);
    // m_GrabberCommand = new Tuning_GrabberCommand(m_grabberSubsystem, 0, m_F310);
    // m_ShoulderCommand = new Tuning_ShoulderCommand(m_ShoulderSubsystem, m_F310);

    configureButtonBindings();

    // Commands requiring buttons
    // m_OperatorSelectorCommand = new
    // OperatorSelectorCommand(OperatorSelectorBackward, OperatorSelectorForward,
    // m_OperatorInterfaceSubsystem);
  }

  public JoystickButton OperatorSelectorForward;
  public JoystickButton OperatorSelectorBackward;

  private void configureButtonBindings() {
    OperatorSelectorForward = new JoystickButton(m_F310, 2);
    OperatorSelectorBackward = new JoystickButton(m_F310, 3);
    // modifying to be handled by the command while we tune the mechanisms...
    // grabber_in_Button.whileTrue(new Tuning_GrabberCommand(m_grabberSubsystem, .2,
    // m_F310));
    // grabber_out_Button.whileTrue(new Tuning_GrabberCommand(m_grabberSubsystem,
    // -0.2, m_F310));
    //
    new JoystickButton(m_F310, Constants.F310.Y).whileTrue(new Manual_ShoulderCommand(m_ShoulderSubsystem, 0.2));
    new JoystickButton(m_F310, Constants.F310.A).whileTrue(new Manual_ShoulderCommand(m_ShoulderSubsystem, -0.2));
    // arm_Button.whileTrue(new TestingArmCommand(m_telescopingSubsystem));
    new JoystickButton(m_F310, Constants.F310.X).whileTrue(new Manual_TelescopeCommand(m_telescopingSubsystem, 0.2));
    new JoystickButton(m_F310, Constants.F310.B).whileTrue(new Manual_TelescopeCommand(m_telescopingSubsystem, -0.2));


    new JoystickButton(m_F310, Constants.F310.RightShoulderButton).whileTrue(new Tuning_ShoulderCommand(m_ShoulderSubsystem));
    // OperatorSelectorForward.onTrue(new OperatorSelectorCommand(true, op));
    // OperatorSelectorBackward.onTrue(new OperatorSelectorCommand(false, op));

  }

  public Command getTeleopCommand() {
    return m_TelescopingCommand;
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
