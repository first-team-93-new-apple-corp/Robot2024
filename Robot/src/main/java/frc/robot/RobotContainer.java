// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.OperatorSelectorCommand;
import frc.robot.commands.Tuning_Commands.Tuning_GrabberCommand;
import frc.robot.commands.Tuning_Commands.Tuning_TelescopeCommand;
import frc.robot.commands.Tuning_Commands.Tuning_ShoulderCommand;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.OperatorInterfaceSub;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class RobotContainer {
  TelescopingSubsystem m_telescopingSubsystem;
  GrabberSubsystem m_grabberSubsystem;
  ShoulderSubsystem m_ShoulderSubsystem;

  Tuning_TelescopeCommand m_TelescopingCommand;
  Tuning_GrabberCommand m_GrabberCommand;
  Tuning_ShoulderCommand m_ShoulderCommand;
  OperatorSelectorCommand m_OperatorSelectorCommand;

  XboxController m_F310;

  JoystickButton grabber_in_Button;
  JoystickButton grabber_out_Button;
  JoystickButton arm_Button;
  JoystickButton arm_In;
  JoystickButton arm_Out;

  // Joysticks

  // Other Definitions
  OperatorInterfaceSub op = new OperatorInterfaceSub();

  public RobotContainer() {

    // Controllers
    m_F310 = new XboxController(0);
    grabber_in_Button = new JoystickButton(m_F310, Constants.F310_A);
    grabber_out_Button = new JoystickButton(m_F310, Constants.F310_B);
    arm_Button = new JoystickButton(m_F310, Constants.F310_Start);
    arm_In = new JoystickButton(m_F310, Constants.F310_X);
    arm_Out = new JoystickButton(m_F310, Constants.F310_Y);

    // Subsystems
    m_telescopingSubsystem = new TelescopingSubsystem();
    m_grabberSubsystem = new GrabberSubsystem();
    m_ShoulderSubsystem = new ShoulderSubsystem();

    // Commands
    m_TelescopingCommand = new Tuning_TelescopeCommand(m_telescopingSubsystem);
    m_GrabberCommand = new Tuning_GrabberCommand(m_grabberSubsystem, 0, m_F310);
    m_ShoulderCommand = new Tuning_ShoulderCommand(m_ShoulderSubsystem, m_F310);

    configureButtonBindings();
    // Commands requiring buttons
    m_OperatorSelectorCommand = new OperatorSelectorCommand( OperatorSelectorBackward, OperatorSelectorForward, op);
  }

  public JoystickButton OperatorSelectorForward;
  public JoystickButton OperatorSelectorBackward;

  private void configureButtonBindings() {
    OperatorSelectorForward = new JoystickButton(m_F310, 2);
    OperatorSelectorBackward = new JoystickButton(m_F310, 3);
    // modifying to be handled by the command while we tune the mechanisms...
    grabber_in_Button.whileTrue(new Tuning_GrabberCommand(m_grabberSubsystem, .2, m_F310));
    grabber_out_Button.whileTrue(new Tuning_GrabberCommand(m_grabberSubsystem, -0.2, m_F310));
    // arm_Button.whileTrue(new TestingArmCommand(m_telescopingSubsystem));
    // arm_In.whileTrue(new ManualTelescopeCommand(m_telescopingSubsystem, -0.1));
    // arm_Out.whileTrue(new ManualTelescopeCommand(m_telescopingSubsystem, 0.1));
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
