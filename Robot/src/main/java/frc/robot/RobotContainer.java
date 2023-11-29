package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OperatorInterfaceSubsystem;
import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.commands.AutonCommands.DriveAndLevel;
import frc.robot.commands.AutonCommands.LockWheels;
import frc.robot.commands.AutonCommands.Test;
import frc.robot.commands.AutonCommands.Circle;
import frc.robot.commands.AutonCommands.Cones;
import frc.robot.commands.AutonCommands.AroundLevel;
import frc.robot.commands.HumanDrive;

public class RobotContainer {
  // Subsystems
  OperatorInterfaceSubsystem m_OperatorInterfaceSubsystem;
  DriveSubsystem m_DriveSubsystem;
  AutonSubsystem m_AutonSubsystem;
  VisionSubsystem m_VisionSubsystem;
  Joystick Driver2;

  // Commands
  HumanDrive m_TeleopDriveCommand;
  // Controllers
  // XboxController m_F310;
  Joystick Driver1;
  Joystick Operator1;
  Joystick Operator2;

  // Buttons
  JoystickButton LockWheels;
  // JoystickButton OperatorSelectorForward;
  // JoystickButton OperatorSelectorBackward;

  // Other Definitions
  SendableChooser<Command> AutonChooser;
  Pigeon2 Pigeon;

  public RobotContainer() {
    // Controllers
    Driver1 = new Joystick(Constants.Joystick_Port.Driver1Port); // Driver 1
    Driver2 = new Joystick(Constants.Joystick_Port.Driver2Port); // Driver 2
    Operator1 = new Joystick(Constants.Joystick_Port.Operator1Port); // Operator 1
    Operator2 = new Joystick(Constants.Joystick_Port.Operator2Port); // Operator 2

    // Subsystems
    m_DriveSubsystem = new DriveSubsystem();
    m_AutonSubsystem = new AutonSubsystem();
    m_VisionSubsystem = new VisionSubsystem("limelight-front");
    // Commands
    m_TeleopDriveCommand = new HumanDrive(m_DriveSubsystem, Driver1, Driver2);
    // Buttons
    LockWheels = new JoystickButton(Driver2, 3);

    // PidgeonAngle
    SmartDashboard.putNumber("Current Pigeon Angle", DriveSubsystem.getHeading());

    // m_OperatorInterfaceSubsystem = new OperatorInterfaceSubsystem();

    // Auton Path Chooser
    AutonChooser = new SendableChooser<Command>();

    AutonChooser.setDefaultOption("No Path", null);
    AutonChooser.addOption(
        "Cones",
        Cones.generatePath(m_AutonSubsystem, m_DriveSubsystem));
    AutonChooser.addOption(
        "Around and Level",
        AroundLevel.generatePath(m_AutonSubsystem, m_DriveSubsystem));
    AutonChooser.addOption(
        "Circle",
        Circle.generatePath(m_AutonSubsystem, m_DriveSubsystem));
    AutonChooser.addOption(
        "Test",
        Test.generatePath(m_AutonSubsystem, m_DriveSubsystem));
        
    SmartDashboard.putData("Auton Chooser", AutonChooser);

    configureButtonBindings();
  }

  public void setTeleopBindings() {
    m_DriveSubsystem.setDefaultCommand(m_TeleopDriveCommand);
    LockWheels.whileTrue(new LockWheels(m_DriveSubsystem));
  }

  public void scheduleTeleopCommands() {
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    System.out.println(AutonChooser.getSelected());
    return AutonChooser.getSelected();
  }
}
