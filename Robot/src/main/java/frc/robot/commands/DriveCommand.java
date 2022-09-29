// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_DriveSubsystem;

  private Joystick m_Joystick;
  private double Joystick_Deadzone = 0.05;

  public DriveCommand(DriveSubsystem m_DriveSubsystem, Joystick m_Joystick) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_Joystick = m_Joystick;
    addRequirements(m_DriveSubsystem);
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
    
    m_DriveSubsystem.drive(x, y, z, false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
