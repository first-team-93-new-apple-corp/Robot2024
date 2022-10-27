// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.lang.reflect.Field;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_DriveSubsystem;

  private XboxController F310;
  private Joystick m_Joystick1;
  private Joystick m_Joystick2;
  private double Joystick_Deadzone = 0.05;

  public int DriveSchematic;
  public int DriveMode;
  public boolean ToggleButton;
  public boolean HeldButton;
  public boolean ButtonReleased;

  enum DriveState {
    DEFAULT_SETUP,
    DEFAULT_STATE,
    HELD_INIT,
    HELD_FIELD_RELATIVE,
    TOGGLE_SETUP,
    TOGGLE_FIELD_RELATIVE_STAGE_1,
    TOGGLE_FIELD_RELATIVE_STAGE_2,
  }
  DriveState Current_Drive_State;

  public DriveCommand(DriveSubsystem m_DriveSubsystem, Joystick m_Joystick1, Joystick m_Joystick2, XboxController F310) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;
    this.F310 = F310;
    SmartDashboard.putNumber("DriveScheme", 0);
    addRequirements(m_DriveSubsystem);

    Current_Drive_State = DriveState.DEFAULT_STATE;
  }

  @Override
  public void initialize() {

  }

  private double x = 0;
  private double y = 0;
  private double z = 0;

  private boolean Toggle = false;



  @Override
  public void execute() {

  HeldButton = m_Joystick1.getRawButton(13);
  ButtonReleased = m_Joystick1.getRawButtonReleased(12);

  DriveSchematic = (int) SmartDashboard.getNumber("DriveScheme", 0);

  System.out.println(Current_Drive_State);

    switch(DriveSchematic){
      case 0:
        x = checkJoystickDeadzone(m_Joystick1.getRawAxis(1));
        y = checkJoystickDeadzone(m_Joystick1.getRawAxis(0));
        z = checkJoystickDeadzone(m_Joystick1.getRawAxis(2));
      ;
      case 1:
        x = checkJoystickDeadzone(m_Joystick1.getRawAxis(1));
        y = checkJoystickDeadzone(m_Joystick1.getRawAxis(0));
        z = checkJoystickDeadzone(m_Joystick2.getRawAxis(0));
      ;
      case 2:
      x = F310.getRightY();
      y = F310.getRightX();
      z = F310.getLeftX();
    ;
    }

    if(DriveSchematic == 0){
      DriveMode = 0;
    }
    else if(DriveSchematic == 1){
      DriveMode = 0;
    }
    else {
      DriveMode = 1;
    }

    //State Machine for Field Relative
  switch (Current_Drive_State) {
    case DEFAULT_SETUP:
        if(ButtonReleased){
          if(Toggle){
            Current_Drive_State = DriveState.DEFAULT_STATE;
          }
        else{
            Current_Drive_State = DriveState.TOGGLE_SETUP;
        }
        }
    break;
    case DEFAULT_STATE:
    Toggle = false;
      if(DriveMode == 0){
        if (HeldButton) {
          Current_Drive_State = DriveState.HELD_INIT;
        } 
        else if (m_Joystick1.getRawButton(12)) {
          Current_Drive_State = DriveState.TOGGLE_SETUP;
        } 
        else {
          m_DriveSubsystem.drive(x, y, z, false);
        }
      }
      else {
        if (F310.getAButton()) {
          if(!Toggle)
          Current_Drive_State = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_2;
        } 
        else {
          m_DriveSubsystem.drive(x, y, z, false);
        }
      }
    break;
    case HELD_INIT:
    m_DriveSubsystem.zeroHeading();
      if (m_Joystick1.getRawButtonReleased(13)) {
        Current_Drive_State = DriveState.DEFAULT_STATE;
      } 
      else {
          Current_Drive_State = DriveState.HELD_FIELD_RELATIVE;
      }
    case HELD_FIELD_RELATIVE:
    m_DriveSubsystem.drive(x, y, z, true);
      if (m_Joystick1.getRawButtonReleased(13)) {
          Current_Drive_State = DriveState.DEFAULT_STATE;
      }
    break;
    case TOGGLE_SETUP:
    m_DriveSubsystem.zeroHeading();
      if (ButtonReleased) {
          Current_Drive_State = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_2;
      } 
      else {
          Current_Drive_State = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_1;
      }
    case TOGGLE_FIELD_RELATIVE_STAGE_1:
    m_DriveSubsystem.drive(x, y, z, true);
      if (ButtonReleased) {
        Current_Drive_State = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_2;
      }
    break;
    case TOGGLE_FIELD_RELATIVE_STAGE_2:
      if(DriveMode == 0){
      m_DriveSubsystem.drive(x, y, z, true);
        if (m_Joystick1.getRawButton(12)) {
          Toggle = true;
          Current_Drive_State = DriveState.DEFAULT_SETUP;
        }
      }
      else {
      m_DriveSubsystem.drive(x, y, z, true);
        if(F310.getAButton()){
          Toggle = true;
          Current_Drive_State = DriveState.DEFAULT_STATE;
        }
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
