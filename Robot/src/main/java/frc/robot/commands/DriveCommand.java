// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.lang.reflect.Field;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_DriveSubsystem;

  private Joystick m_Joystick;
  public int button;
  private double Joystick_Deadzone = 0.05;

  enum DriveState {
    DEFAULT_STATE,
    HELD_INIT,
    HELD_FIELD_RELATIVE,
    TOGGLE_SETUP,
    TOGGLE_FIELD_RELATIVE_STAGE_1,
    TOGGLE_FIELD_RELATIVE_STAGE_2,
  }

  DriveState Current_Drive_State;

  public DriveCommand(DriveSubsystem m_DriveSubsystem, Joystick m_Joystick) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_Joystick = m_Joystick;
    addRequirements(m_DriveSubsystem);

    Current_Drive_State = DriveState.DEFAULT_STATE;
  }

  @Override
  public void initialize() {
  }

  private double x = 0;
  private double y = 0;
  private double z = 0;
  // public double recordY = 0;
  // public double recordX = 0;
  // public double recordZ = 0;

  private boolean Toggle_Start = true;
  private boolean Field_Relative_Released = true;
  private boolean Field_Relative_Activated = false;

  @Override
  public void execute() {
    // m_DriveSubsystem.getEncoderValues();

    x = checkJoystickDeadzone(m_Joystick.getRawAxis(1));
    y = checkJoystickDeadzone(m_Joystick.getRawAxis(0));
    z = checkJoystickDeadzone(m_Joystick.getRawAxis(2));

    // BUTTON 13 HELD FIELD RELATIVE
    // BUTTON 12 TOGGLE FIELD RELATIVE

    switch (Current_Drive_State) {
      case DEFAULT_STATE:
        if (m_Joystick.getRawButton(13)) {
          Current_Drive_State = DriveState.HELD_INIT;
        } else if (m_Joystick.getRawButton(12)) {
          Current_Drive_State = DriveState.TOGGLE_SETUP;
        } else {
          m_DriveSubsystem.drive(x, y, z, false);
        }
      case HELD_INIT:
        m_DriveSubsystem.zeroHeading();
        if (m_Joystick.getRawButtonReleased(13)) {
          Current_Drive_State = DriveState.DEFAULT_STATE;
        } else {
          Current_Drive_State = DriveState.HELD_FIELD_RELATIVE;
        }
      case HELD_FIELD_RELATIVE:
        m_DriveSubsystem.drive(x, y, z, true);
        if (m_Joystick.getRawButtonReleased(13)) {
          Current_Drive_State = DriveState.DEFAULT_STATE;
        }
        break;
      case TOGGLE_SETUP:
        m_DriveSubsystem.zeroHeading();

        if (m_Joystick.getRawButtonReleased(12)) {
          Current_Drive_State = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_2;
        } else {
          Current_Drive_State = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_1;
        }
      case TOGGLE_FIELD_RELATIVE_STAGE_1:
        m_DriveSubsystem.drive(x, y, z, true);
        if (m_Joystick.getRawButtonReleased(12)) {
          Current_Drive_State = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_2;
        }
        break;
      case TOGGLE_FIELD_RELATIVE_STAGE_2:
        m_DriveSubsystem.drive(x, y, z, true);
        if (m_Joystick.getRawButton(12)) {
          Current_Drive_State = DriveState.DEFAULT_STATE;
        }
        break;
    }
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
