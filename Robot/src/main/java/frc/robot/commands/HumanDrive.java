// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CustomRotationHelper;
import frc.robot.subsystems.DriveSubsystem;

public class HumanDrive extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private enum DriveModes {
    One_Stick_Drive,
    Two_Stick_Drive,
    F310_Drive,
    F310_Drive_Inverted,
    F310_TurningBumpers
  }

  private DriveSubsystem m_DriveSubsystem;
  private CustomRotationHelper rotationHelper;

  private XboxController m_F310;
  private Joystick m_Joystick1;
  private Joystick m_Joystick2;

  private boolean ToggleButton;
  private boolean HeldButton;
  private boolean ToggleButtonReleased = false;
  private boolean HeldButtonReleased = false;

  private double Joystick_Deadzone = 0.04;
  private double Controller_Deadzone = 0.2;

  private double x = 0;
  private double y = 0;
  private double z = 0;

  private DriveModes CurrentDriveMode;
  private DriveModes LastDriveMode = DriveModes.One_Stick_Drive;

  private static SendableChooser<DriveModes> DriveModeChooser;

  /**
   * Teleop Drive Command
   * 
   * @param m_DriveSubsystem Auton Subsystem
   * @param m_Joystick1      The Main Joystick
   * @param m_Joystick2      A Secondary Joystick
   * @param m_F310           Xbox Controller
   *
   */
  public HumanDrive(
      DriveSubsystem m_DriveSubsystem,
      Joystick m_Joystick1,
      Joystick m_Joystick2,
      XboxController m_F310) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;
    this.m_F310 = m_F310;

    rotationHelper = new CustomRotationHelper(m_Joystick1);


    try {
      SmartDashboard.getData("DriveScheme");
    } catch (Exception e) {
      DriveModeChooser = new SendableChooser<DriveModes>();



      DriveModeChooser.setDefaultOption("2 Stick Drive", DriveModes.Two_Stick_Drive);
      DriveModeChooser.addOption(
          "1 Stick Drive",
          DriveModes.One_Stick_Drive);
      DriveModeChooser.addOption("F310 Drive", DriveModes.F310_Drive);
      DriveModeChooser.addOption("F310 Inverted", DriveModes.F310_Drive_Inverted);
      DriveModeChooser.addOption("F310_TurningBumpers", DriveModes.F310_TurningBumpers);

      SmartDashboard.putData("Drive Scheme", DriveModeChooser);
    }

    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Drive();
  }

  /**
   * Runs the drive based on the Drive Mode and joystick values
   */
  public void Drive() {
    CurrentDriveMode = DriveModeChooser.getSelected();

    if (CurrentDriveMode != LastDriveMode) {
      m_DriveSubsystem.resetDriveStateMachine();
    }

    LastDriveMode = CurrentDriveMode;

    switch (CurrentDriveMode) {
      // one stick driving
      case One_Stick_Drive:
        x = checkJoystickDeadzone(m_Joystick1.getRawAxis(1), Joystick_Deadzone);
        y = 0;
        // y = checkJoystickDeadzone(m_Joystick1.getRawAxis(0), Joystick_Deadzone);
        z = checkJoystickDeadzone(m_Joystick1.getRawAxis(2), Joystick_Deadzone);

        HeldButton = m_Joystick1.getRawButton(13);
        HeldButtonReleased = m_Joystick1.getRawButtonReleased(13);

        ToggleButton = m_Joystick1.getRawButton(12);
        ToggleButtonReleased = m_Joystick1.getRawButtonReleased(12);

        
        break;

      // two stick driving
      case Two_Stick_Drive:
        x = checkJoystickDeadzone(m_Joystick1.getRawAxis(1), Joystick_Deadzone);
        y = checkJoystickDeadzone(m_Joystick1.getRawAxis(0), Joystick_Deadzone);

        z = checkJoystickDeadzone(m_Joystick2.getRawAxis(0), Joystick_Deadzone);

        HeldButton = m_Joystick1.getRawButton(13);
        HeldButtonReleased = m_Joystick1.getRawButtonReleased(13);

        ToggleButton = m_Joystick1.getRawButton(12);
        ToggleButtonReleased = m_Joystick1.getRawButtonReleased(12);

        break;
      // F310 Drive
      case F310_Drive:
        x = checkJoystickDeadzone(m_F310.getRightY(), Controller_Deadzone);
        x = Math.pow(x, 2) * Math.signum(x);

        y = checkJoystickDeadzone(m_F310.getRightX(), Controller_Deadzone);
        y = Math.pow(y, 2) * Math.signum(y);

        z = checkJoystickDeadzone(m_F310.getLeftX(), Controller_Deadzone);

        HeldButton = m_F310.getLeftBumper();
        HeldButtonReleased = m_F310.getLeftBumperReleased();

        ToggleButton = m_F310.getRightBumper();
        ToggleButtonReleased = m_F310.getRightBumperReleased();
        break;
      case F310_Drive_Inverted:
        x = checkJoystickDeadzone(m_F310.getLeftY(), Controller_Deadzone);
        x = Math.pow(x, 2) * Math.signum(x);

        y = checkJoystickDeadzone(m_F310.getLeftX(), Controller_Deadzone);
        y = Math.pow(y, 2) * Math.signum(y);

        z = checkJoystickDeadzone(m_F310.getRightX(), Controller_Deadzone);

        HeldButton = m_F310.getLeftBumper();
        HeldButtonReleased = m_F310.getLeftBumperReleased();

        ToggleButton = m_F310.getRightBumper();
        ToggleButtonReleased = m_F310.getRightBumperReleased();
        break;
      case F310_TurningBumpers:
      x = checkJoystickDeadzone(m_F310.getLeftY(), Controller_Deadzone);
      x = Math.pow(x, 2) * Math.signum(x);

      y = checkJoystickDeadzone(m_F310.getLeftX(), Controller_Deadzone);
      y = Math.pow(y, 2) * Math.signum(y);

      z = checkJoystickDeadzone(m_F310.getLeftTriggerAxis()*-1+m_F310.getRightTriggerAxis(), 0.05);

      HeldButton = m_F310.getLeftBumper();
      HeldButtonReleased = m_F310.getLeftBumperReleased();

      ToggleButton = m_F310.getRightBumper();
      ToggleButtonReleased = m_F310.getRightBumperReleased();
        break;
    }

    m_DriveSubsystem.DriveStateMachine(
        -(x),
        -(y),
        -(z),
        HeldButton,
        HeldButtonReleased,
        ToggleButton,
        ToggleButtonReleased,
        rotationHelper.povButton());
  }

  /**
   * Checks if the joystick is within the deadzone
   *
   * @param joystickValue: the value of the joystick
   * @return the joystick value if it is outside the deadzone, 0 if it is within
   *         the dead zones
   */
  public double checkJoystickDeadzone(double joystickValue, double deadzone) {
    if (Math.abs(joystickValue) < deadzone) {
      return 0.0;
    } else {
      return joystickValue;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
