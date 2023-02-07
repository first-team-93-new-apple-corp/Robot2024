package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonCommands.CableBumpBlue1Pickup;
import frc.robot.commands.AutonCommands.DriveAndLevel;
import frc.robot.commands.OperatorSelectorCommand;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OperatorInterfaceSubsystem;

public class RobotContainer {

  // Subsystems
  OperatorInterfaceSubsystem m_OperatorInterfaceSubsystem;
  DriveSubsystem m_DriveSubsystem;
  AutonSubsystem m_AutonSubsystem;

  // Commands

  // Controllers
  // XboxController m_F310;
  Joystick Driver1;
  Joystick Driver2;
  Joystick Operator1;
  Joystick Operator2;

  // Buttons
  JoystickButton grabber_in_Button;
  JoystickButton grabber_out_Button;
  JoystickButton arm_Button;
  JoystickButton arm_In;
  JoystickButton arm_Out;
  JoystickButton TuningWrist;

  // JoystickButton OperatorSelectorForward;
  // JoystickButton OperatorSelectorBackward;

  // Other Definitions
  SendableChooser<Command> AutonChooser;

  public RobotContainer() {
    // Controllers
    Driver1 = new Joystick(Constants.Joystick_Port.Driver1Port); // Driver 1
    Driver2 = new Joystick(Constants.Joystick_Port.Driver2Port); // Driver 2
    Operator1 = new Joystick(Constants.Joystick_Port.Operator1Port); // Operator 1
    Operator2 = new Joystick(Constants.Joystick_Port.Operator2Port); // Operator 2

    // Joystick Buttons

    // grabber_in_Button = new JoystickButton(m_F310, Constants.F310.A);
    // grabber_out_Button = new JoystickButton(m_F310, Constants.F310.B);
    // arm_Button = new JoystickButton(m_F310, Constants.F310.Start);
    // arm_In = new JoystickButton(m_F310, Constants.F310.X);
    // arm_Out = new JoystickButton(m_F310, Constants.F310.Y);

    // Subsystems
    m_DriveSubsystem = new DriveSubsystem();
    m_AutonSubsystem = new AutonSubsystem();
    // m_OperatorInterfaceSubsystem = new OperatorInterfaceSubsystem();

    // Commands

    // m_Manual_ShoulderCommand = new Manual_ShoulderCommand(m_ShoulderSubsystem,
    // 0.1, m_F310);
    // m_Manual_GrabberCommand = new Manual_GrabberCommand(m_grabberSubsystem, 0.1,
    // m_F310);
    // m_Manual_WristCommand = new Manual_WristCommand(m_WristSubsystem, 0.1,
    // m_F310);
    // m_TelescopingCommand = new Tuning_TelescopeCommand(m_telescopingSubsystem);
    // m_GrabberCommand = new Tuning_GrabberCommand(m_grabberSubsystem, 0, m_F310);
    // m_ShoulderCommand = new Tuning_ShoulderCommand(m_ShoulderSubsystem, m_F310);
    // m_OperatorSelectorCommand = new OperatorSelectorCommand(OperatorSelectorBackward, OperatorSelectorForward,
    // m_OperatorInterfaceSubsystem);

    // Auton Path Chooser
    AutonChooser = new SendableChooser<Command>();

    AutonChooser.setDefaultOption("No Path", null);
    AutonChooser.addOption(
      "Test Path",
      DriveAndLevel.generatePath(m_AutonSubsystem, m_DriveSubsystem)
    );
    AutonChooser.addOption(
      "CableBumpBlue1Pickup",
      CableBumpBlue1Pickup.generatePath(m_AutonSubsystem, m_DriveSubsystem)
    );
    SmartDashboard.putData("Auton Chooser", AutonChooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {


    // TuningWrist = new JoystickButton(m_F310, Constants.F310.Start);
    // TuningWrist.whileTrue(new Tuning_WristCommand(m_WristSubsystem));
    // OperatorSelectorForward = new JoystickButton(m_F310, 2);
    // OperatorSelectorBackward = new JoystickButton(m_F310, 3);
    // modifying to be handled by the command while we tune the mechanisms...
    // grabber_in_Button.whileTrue(new Tuning_GrabberCommand(m_grabberSubsystem, .2,
    // m_F310));
    // grabber_out_Button.whileTrue(new Tuning_GrabberCommand(m_grabberSubsystem,
    // -0.2, m_F310));
    //
    // arm_Button.whileTrue(new TestingArmCommand(m_telescopingSubsystem));

    // Shoulder

    // new JoystickButton(m_F310, Constants.F310.Y).whileTrue(new
    // Manual_ShoulderCommand(m_ShoulderSubsystem, 0.1, m_F310));
    // new JoystickButton(m_F310, Constants.F310.A).whileTrue(new
    // Manual_ShoulderCommand(m_ShoulderSubsystem, -0.1, m_F310));
    // new JoystickButton(m_F310, Constants.F310.RightShoulderButton)
    // .whileTrue(new Tuning_ShoulderCommand(m_ShoulderSubsystem));

    // Telescope

    // new JoystickButton(m_F310, Constants.F310.RightShoulderButton).whileTrue(new
    // Manual_TelescopeCommand(m_telescopingSubsystem, 0.2));
    // new JoystickButton(m_F310, Constants.F310.LeftShoulderButton).whileTrue(new
    // Manual_TelescopeCommand(m_telescopingSubsystem, -0.2));

    // Wrist

    // new JoystickButton(m_F310, Constants.F310.X).whileTrue(new
    // Manual_WristCommand(m_WristSubsystem, 0.15));
    // new JoystickButton(m_F310, Constants.F310.B).whileTrue(new
    // Manual_WristCommand(m_WristSubsystem, -0.15));

    // Grabber

    // new JoystickButton(m_F310, Constants.F310.Start).whileTrue(new
    // Manual_GrabberCommand(m_grabberSubsystem, 0.1));
    // new JoystickButton(m_F310, Constants.F310.Back).whileTrue(new
    // Manual_GrabberCommand(m_grabberSubsystem, -0.1));

    // OperatorSelectorForward.onTrue(new OperatorSelectorCommand(true, op));
    // OperatorSelectorBackward.onTrue(new OperatorSelectorCommand(false, op));

  }

  public Command getAutonomousCommand() {
    return null;
  }

}
