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

public class DriveCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private enum DriveModes {
    One_Stick_Drive,
    Two_Stick_Drive,
    F310_Drive,
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

  private double x = 0;
  private double y = 0;
  private double z = 0;

  private DriveModes CurrentDriveMode;
  private DriveModes LastDriveMode = DriveModes.One_Stick_Drive;

  private SendableChooser<DriveModes> DriveModeChooser;

  /**
   * Teleop Drive Command
   * @end No End Condition
   * @param m_DriveSubsystem Drive Subsystem
   * @param m_Joystick1 Joystick 1 (the drive joystick)
   * @param m_Joystick2 Joystick 2 (the turning joystick)
   * @param F310 F310 Controller
   * 
   */
  public DriveCommand(
      DriveSubsystem m_DriveSubsystem,
      Joystick m_Joystick1,
      Joystick m_Joystick2,
      XboxController m_F310
      ) {

    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;
    this.m_F310 = m_F310;
  
    DriveModeChooser = new SendableChooser<DriveModes>();
    
    rotationHelper = new CustomRotationHelper(m_Joystick1);

    DriveModeChooser.setDefaultOption(
        "1 Stick Drive",
        DriveModes.One_Stick_Drive);

    DriveModeChooser.addOption(
        "2 Stick Drive",
        DriveModes.Two_Stick_Drive);

    DriveModeChooser.addOption("F310 Drive", DriveModes.F310_Drive);

    SmartDashboard.putData("Drive Scheme", DriveModeChooser);

    addRequirements(m_DriveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    HumanDrive();
    // OTFAuto();
  }


  /**
   * A regular drive mode that uses the joysticks to drive the robot.
   * 
   */
  public void HumanDrive(){
    CurrentDriveMode = DriveModeChooser.getSelected();

    if (CurrentDriveMode != LastDriveMode) {
      m_DriveSubsystem.resetDriveStateMachine();
    }

    LastDriveMode = CurrentDriveMode;

    switch (CurrentDriveMode) {

      // one stick driving
      case One_Stick_Drive:
        x = checkJoystickDeadzone(m_Joystick1.getRawAxis(1));
        y = checkJoystickDeadzone(m_Joystick1.getRawAxis(0));
        z = checkJoystickDeadzone(m_Joystick1.getRawAxis(2));

        HeldButton = m_Joystick1.getRawButton(13);
        HeldButtonReleased = m_Joystick1.getRawButtonReleased(13);

        ToggleButton = m_Joystick1.getRawButton(12);
        ToggleButtonReleased = m_Joystick1.getRawButtonReleased(12);

        break;

      // two stick driving
      case Two_Stick_Drive:
        x = checkJoystickDeadzone(m_Joystick1.getRawAxis(1));
        y =checkJoystickDeadzone(m_Joystick1.getRawAxis(0));
         
        z = checkJoystickDeadzone(m_Joystick2.getRawAxis(0));

        HeldButton = m_Joystick1.getRawButton(13);
        HeldButtonReleased = m_Joystick1.getRawButtonReleased(13);

        ToggleButton = m_Joystick1.getRawButton(12);
        ToggleButtonReleased = m_Joystick1.getRawButtonReleased(12);

        break;

      // F310 Drive
      case F310_Drive:
        x = m_F310.getRightY();
        x = Math.pow(x, 2) * Math.signum(x) ;

        y = m_F310.getRightX();
        y = Math.pow(y, 2) * Math.signum(y);

        z = m_F310.getLeftX();

        HeldButton = m_F310.getLeftBumper();
        HeldButtonReleased = m_F310.getLeftBumperReleased();

        ToggleButton = m_F310.getRightBumper();
        ToggleButtonReleased = m_F310.getRightBumperReleased();
        break;

    }

    m_DriveSubsystem.DriveStateMachine(
        -(x), -(y),
        -(z),
        HeldButton,
        HeldButtonReleased,
        ToggleButton,
        ToggleButtonReleased,
        rotationHelper.povButton(),
        getMaxSpeedMultiplier(m_Joystick1));
  }

    /**
   * Speed multiplier based on Joystick Slider
   * 
   * @deprecated will be removed and max speed allowed instead or a slower speed mode toggle.  
   * @param Joystick: the Joystick
   * @return the calculated speed multiplier
   */
  public double getMaxSpeedMultiplier(Joystick Joystick){
    double slideAxis = -Joystick.getRawAxis(3);
    double output = ((slideAxis+1)/8)+0.25;

    return 1;
  }

  /**
   * Checks if the joystick is within the deadzone
   * 
   * @param joystickValue: the value of the joystick
   * @return the joystick value if it is outside the deadzone, 0 if it is within the dead zones
   */
  public double checkJoystickDeadzone(double joystickValue) {
    if (Math.abs(joystickValue) < Joystick_Deadzone) {
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
