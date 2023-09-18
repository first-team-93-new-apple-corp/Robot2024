// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveConstants;
// import frc.robot.CustomRotationHelper;
import frc.robot.subsystems.DriveSubsystem;

public class HumanDrive extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private enum DriveModes {
    One_Stick_Drive,
    Two_Stick_Drive,
    Testing_Drive
  }

  private DriveSubsystem m_DriveSubsystem;
  // private CustomRotationHelper rotationHelper;

  private Joystick m_Joystick1;
  private Joystick m_Joystick2;

  private boolean ToggleButton;
  private boolean HeldButton;
  private boolean ToggleButtonReleased = false;
  private boolean HeldButtonReleased = false;

  private double Joystick_Deadzone = 0.07;

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
      Joystick m_Joystick2) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;

    // rotationHelper = new CustomRotationHelper(m_Joystick1);


    try {
      SmartDashboard.getData("DriveScheme");
    } catch (Exception e) {
      DriveModeChooser = new SendableChooser<DriveModes>();



      DriveModeChooser.setDefaultOption("2 Stick Drive", DriveModes.Two_Stick_Drive);
      DriveModeChooser.addOption(
          "1 Stick Drive",
          DriveModes.One_Stick_Drive);
      DriveModeChooser.addOption("Testing", DriveModes.Testing_Drive);
      // DriveModeChooser.addOption("F310 Inverted", DriveModes.F310_Drive_Inverted);
      // DriveModeChooser.addOption("F310_TurningBumpers", DriveModes.F310_TurningBumpers);

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

        POVButton pov0 = new POVButton(m_Joystick2, 0); // front
        POVButton pov45 = new POVButton(m_Joystick2, 45); // fr wheel
        POVButton pov90 = new POVButton(m_Joystick2, 90); // right
        POVButton pov135 = new POVButton(m_Joystick2, 135); // bl wheel
        POVButton pov180 = new POVButton(m_Joystick2, 180); // back
        POVButton pov225 = new POVButton(m_Joystick2, 225);// bl wheel
        POVButton pov270 = new POVButton(m_Joystick2, 270);// left
        POVButton pov315 = new POVButton(m_Joystick2, 315); // fl wheel
        
        if(pov0.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Front;
        } else if(pov45.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Location_FR;
        } else if(pov90.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Right;
        } else if(pov135.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Location_BR;
        } else if(pov180.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Back;
        } else if(pov225.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Location_BL;
        } else if(pov270.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Left;
        } else if(pov315.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Location_FL;
        } else{
            DriveConstants.dCenter = new Translation2d(0,0);
        }

        break;
      case Testing_Drive:
        break; 
    }

    m_DriveSubsystem.DriveStateMachine(
        -(x),
        -(y),
        -(z),
        HeldButton,
        HeldButtonReleased,
        ToggleButton,
        ToggleButtonReleased);
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
