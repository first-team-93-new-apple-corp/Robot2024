// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private enum DriveModes {
    One_Stick_Drive,
    Two_Stick_Drive,
    F310_Drive,
  }

  private DriveSubsystem m_DriveSubsystem;

  private XboxController F310;
  private Joystick m_Joystick1;
  private Joystick m_Joystick2;

  private boolean ToggleButton;
  private boolean HeldButton;
  private boolean ToggleButtonReleased = false;
  private boolean HeldButtonReleased = false;

  private double Joystick_Deadzone = 0.05;

  private double x = 0;
  private double y = 0;
  private double z = 0;

  public DriveModes CurrentDriveMode;
  private DriveModes LastDriveMode = DriveModes.One_Stick_Drive;

  private SendableChooser<DriveModes> DriveModeChooser;

  public DriveCommand(
    DriveSubsystem m_DriveSubsystem,
    Joystick m_Joystick1,
    Joystick m_Joystick2,
    XboxController F310
  ) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;
    this.F310 = F310;

    DriveModeChooser.setDefaultOption(
      "1 Stick Drive",
      DriveModes.One_Stick_Drive
    );
    DriveModeChooser.setDefaultOption(
      "2 Stick Drive",
      DriveModes.Two_Stick_Drive
    );
    DriveModeChooser.setDefaultOption("F310 Drive", DriveModes.F310_Drive);
    SmartDashboard.putData("Drive Scheme", DriveModeChooser);
    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    CurrentDriveMode = DriveModeChooser.getSelected();

    if (CurrentDriveMode != LastDriveMode) {
      m_DriveSubsystem.resetDriveMode();
    }

    LastDriveMode = CurrentDriveMode;

    switch (CurrentDriveMode) {
      // regular drive
      case One_Stick_Drive:
        x = checkJoystickDeadzone(m_Joystick1.getRawAxis(1));
        y = checkJoystickDeadzone(m_Joystick1.getRawAxis(0));
        z = checkJoystickDeadzone(m_Joystick1.getRawAxis(2));

        HeldButton = m_Joystick1.getRawButton(13);
        HeldButtonReleased = m_Joystick1.getRawButtonReleased(13);

        ToggleButton = m_Joystick1.getRawButton(12);
        ToggleButtonReleased = m_Joystick1.getRawButtonReleased(12);

        break;
      // drive with 2 sticksRegularDrive
      case Two_Stick_Drive:
        x = checkJoystickDeadzone(m_Joystick1.getRawAxis(1));
        y = checkJoystickDeadzone(m_Joystick1.getRawAxis(0));
        z = checkJoystickDeadzone(m_Joystick2.getRawAxis(0));

        HeldButton = m_Joystick1.getRawButton(13);
        HeldButtonReleased = m_Joystick1.getRawButtonReleased(13);

        ToggleButton = m_Joystick1.getRawButton(12);
        ToggleButtonReleased = m_Joystick1.getRawButtonReleased(12);

        break;
      // F310 Drive
      case F310_Drive:
        x = F310.getRightY();
        y = F310.getRightX();
        z = F310.getLeftX();


        HeldButton = F310.getAButton();
        HeldButtonReleased = F310.getAButtonReleased();

        ToggleButton = F310.getBButton();
        ToggleButtonReleased = F310.getBButtonReleased();
        break;
    }

    m_DriveSubsystem.DriveStateMachine(
      x,
      y,
      z,
      ToggleButton,
      ToggleButtonReleased,
      HeldButton,
      HeldButtonReleased
    );
  }

  public double checkJoystickDeadzone(double joystickValue) {
    if (Math.abs(joystickValue) < Joystick_Deadzone) {
      return 0.0;
    } else {
      return joystickValue;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
