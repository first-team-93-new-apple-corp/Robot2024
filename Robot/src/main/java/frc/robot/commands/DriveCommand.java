// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_DriveSubsystem;

  private Joystick m_Joystick;
  public int button;
  private double Joystick_Deadzone = 0.05;

  enum DriveState {
    REGULAR_DRIVE,
    TOGGLE_SETUP,
    TOGGLE_FIELD_RELATIVE,
    // FULL_FIELD_RELATIVE,
  }

  DriveState Current_Drive_State;

  public DriveCommand(DriveSubsystem m_DriveSubsystem, Joystick m_Joystick) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_Joystick = m_Joystick;
    addRequirements(m_DriveSubsystem);

    Current_Drive_State = DriveState.REGULAR_DRIVE;
  }

  @Override
  public void initialize() {
    // encoders should be absolute so no need to reset them except for once?
    // test encoder values
    // reset them right before matches?
    // will require testing to figure out
    // m_DriveSubsystem.resetEncoders();

  }

  private double x = 0;
  private double y = 0;
  private double z = 0;
  public double recordY = 0;
  public double recordX = 0;
  public double recordZ = 0;

  public boolean Toggle_Start = true; 
  public boolean Field_Relative_Released = true; 
  boolean Field_Relative_Activated = false; 
  @Override
  public void execute() {
    // m_DriveSubsystem.getEncoderValues();

    x = m_Joystick.getRawAxis(1);
    if (Math.abs(x) < Joystick_Deadzone) {
      x = 0.00;
    } else {
      recordX = x;
    }

    y = m_Joystick.getRawAxis(0);
    if (Math.abs(y) < Joystick_Deadzone) {
      y = 0.0;
    }

    z = m_Joystick.getRawAxis(2);
    if (Math.abs(z) < Joystick_Deadzone) {
      z = 0;
    }

    if (m_Joystick.getRawButton(13)) {
        if(Toggle_Start){
          Toggle_Start = false; 
          Current_Drive_State = DriveState.TOGGLE_SETUP;
        }
    }
    else if(!Field_Relative_Activated){
      Toggle_Start = true; 
      Current_Drive_State = DriveState.REGULAR_DRIVE;
    }


    if(m_Joystick.getRawButtonReleased(12)){
      System.out.println("Button Released");
      Field_Relative_Released = true; 
    }

    if (m_Joystick.getRawButton(12) && Field_Relative_Released) {
      Field_Relative_Activated = true; 
      if (Current_Drive_State == DriveState.TOGGLE_FIELD_RELATIVE) {
        System.out.println("Regular Drive Relative Activated");
        Current_Drive_State = DriveState.REGULAR_DRIVE;
        Field_Relative_Released = true; 
        Field_Relative_Activated = false; 
      } else {
        System.out.println("Field Relative Activated");
        Field_Relative_Released = false; 
        Current_Drive_State = DriveState.TOGGLE_SETUP;
        // m_DriveSubsystem.zeroHeading();
      }
    }



    System.out.println(Current_Drive_State);

    switch (Current_Drive_State) {
      case REGULAR_DRIVE:
        m_DriveSubsystem.drive(x, y, z, false);
        break;
      case TOGGLE_SETUP:
        m_DriveSubsystem.zeroHeading();
        Current_Drive_State = DriveState.TOGGLE_FIELD_RELATIVE;
      case TOGGLE_FIELD_RELATIVE:
        m_DriveSubsystem.drive(x, y, z, true);
        break;
      
      // case FULL_FIELD_RELATIVE:
      //   m_DriveSubsystem.drive(x, y, z, true);
      //   break;




    // System.out.println(m_Joystick.getRawButton(13));


    }
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
