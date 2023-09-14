package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonCommands.CableBumpBlue1Pickup;
import frc.robot.commands.AutonCommands.DriveAndLevel;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CustomRotationHelper;
import frc.robot.subsystems.OperatorInterfaceSubsystem;
import frc.robot.commands.HumanDrive;

public class RobotContainer {

  // Subsystems
  OperatorInterfaceSubsystem m_OperatorInterfaceSubsystem;
  DriveSubsystem m_DriveSubsystem;
  AutonSubsystem m_AutonSubsystem;
  Joystick Driver2;
  // Commands
  CustomRotationHelper m_CustomRotationHelper;
  HumanDrive m_TeleopDriveCommand;

  // Controllers
  // XboxController m_F310;
  Joystick Driver1;
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

    // Subsystems
    m_DriveSubsystem = new DriveSubsystem();
    m_AutonSubsystem = new AutonSubsystem();
    m_CustomRotationHelper = new CustomRotationHelper(Driver2);

    // Commands
    m_TeleopDriveCommand = new HumanDrive(m_DriveSubsystem, Driver1, Driver2);

    // m_OperatorInterfaceSubsystem = new OperatorInterfaceSubsystem();

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

  public void setTeleopBindings() {
    m_DriveSubsystem.setDefaultCommand(m_TeleopDriveCommand);
  }

  public void scheduleTeleopCommands() {}

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }

}
