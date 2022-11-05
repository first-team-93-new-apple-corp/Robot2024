// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.CustomRotationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private enum DriveModes {
    One_Stick_Drive,
    Two_Stick_Drive,
    F310_Drive,
  }

  private DriveSubsystem m_DriveSubsystem;
  private VisionSubsystem m_VisionSubsystem;
  private CustomRotationHelper rotationHelper;

  private XboxController F310;
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

  public DriveModes CurrentDriveMode;
  private DriveModes LastDriveMode = DriveModes.One_Stick_Drive;

  private SendableChooser<DriveModes> DriveModeChooser;
  private POVButton POV;

  public DriveCommand(
      DriveSubsystem m_DriveSubsystem,
      Joystick m_Joystick1,
      Joystick m_Joystick2,
      XboxController F310
      // ,VisionSubsystem m_VisionSubsystem
      ) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    // this.m_VisionSubsystem = m_VisionSubsystem;
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;
    this.F310 = F310;
    
    rotationHelper = new CustomRotationHelper(m_Joystick2);
    
    // SmartDashboard Declarations
    DriveModeChooser = new SendableChooser<DriveModes>();
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
    OTFAuto();
  }
  public void HumanDrive(){
    CurrentDriveMode = DriveModeChooser.getSelected();
    double SpeedMult = getMaxSpeedMultiplier(m_Joystick1);
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
        x = Math.pow(x, 2) * Math.signum(x) ;

        y = F310.getRightX();
        y = Math.pow(y, 2) * Math.signum(y);

        z = F310.getLeftX();

        HeldButton = F310.getLeftBumper();
        HeldButtonReleased = F310.getLeftBumperReleased();

        ToggleButton = F310.getRightBumper();
        ToggleButtonReleased = F310.getRightBumperReleased();
        break;
    }

    m_DriveSubsystem.DriveStateMachine(
        -x, -y,
        -z,
        HeldButton,
        HeldButtonReleased,
        ToggleButton,
        ToggleButtonReleased,
        rotationHelper.povButton(),
        SpeedMult);
  }
  public double getMaxSpeedMultiplier(Joystick Joystick){
    double slideAxis = -Joystick.getRawAxis(3);
    double output = ((slideAxis+1)/4)+0.25;
    return output;
  }
  public void OTFAuto(){ // OTF has to be here so we can get odometry without double calling
    m_VisionSubsystem.getTrajectory(0,0, m_DriveSubsystem.getPose());

  }
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
