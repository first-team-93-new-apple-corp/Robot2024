// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveConstants;
// import frc.robot.CustomRotationHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveState;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;

public class HumanDrive extends Command {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private enum DriveModes {
    One_Stick_Drive,
    Two_Stick_Drive,
    Follow_Tape,
    JoyCon
  }

  private DriveSubsystem m_DriveSubsystem;
  private VisionSubsystem m_VisionSubsystem;
  // private CustomRotationHelper rotationHelper;
  private TimeOfFlight tof = new TimeOfFlight(14);
  private Joystick m_Joystick1;
  private Joystick m_Joystick2;
  // private XboxController m_JoyCons = new XboxController(0);
  private boolean ToggleButton;
  private boolean HeldButton;
  private boolean ToggleButtonReleased = false;
  private boolean HeldButtonReleased = false;

  private double Joystick_Deadzone = 0.07;
  boolean Limit = true;
  boolean chooserToggle = false;

  private double x = 0;
  private double y = 0;
  private double z = 0;
  private int angle;
  private double prevAngle;
  private DriveModes CurrentDriveMode;
  private DriveModes LastDriveMode = DriveModes.One_Stick_Drive;

  private PIDController correctionPID = new PIDController(0.005, 0, 0);
  private static SendableChooser<DriveModes> DriveModeChooser;

  /**
   * Teleop Drive Command
   * 
   * @param m_DriveSubsystem Auton Subsystem
   * @param m_Joystick1      The Main Joystick
   * @param m_Joystick2      A Secondary Joystick
   * @param m_JoyCons        Joy Con controller mapped to an xbox controller
   *                         (scuffed)
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
    m_VisionSubsystem = new VisionSubsystem("limelight-front", m_Joystick1);
        
    try {
      SmartDashboard.getData("DriveScheme");
    } catch (Exception e) {
      DriveModeChooser = new SendableChooser<DriveModes>();

      DriveModeChooser.setDefaultOption("2 Stick Drive", DriveModes.Two_Stick_Drive);
      DriveModeChooser.addOption(
          "1 Stick Drive",
          DriveModes.One_Stick_Drive);
      DriveModeChooser.addOption("Testing", DriveModes.Follow_Tape);
      DriveModeChooser.addOption("JoyCon", DriveModes.JoyCon);
      // DriveModeChooser.addOption("F310 Inverted", DriveModes.F310_Drive_Inverted);
      // DriveModeChooser.addOption("F310_TurningBumpers",
      // DriveModes.F310_TurningBumpers);

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

  public void roundAngle(int angle) {
    if (angle >= 337.5 || angle < 22.5) {
      angle = 0;
    } else if (angle >= 22.5 && angle < 67.5) {
      angle = 45;
    } else if (angle >= 67.5 && angle < 112.5) {
      angle = 90;
    } else if (angle >= 112.5 && angle < 157.5) {
      angle = 135;
    } else if (angle >= 157.5 && angle < 202.5) {
      angle = 180;
    } else if (angle >= 202.5 && angle < 247.5) {
      angle = 225;
    } else if (angle >= 247.5 && angle < 292.5) {
      angle = 270;
    } else if (angle >= 292.5 && angle < 337.5) {
      angle = 315;
    }
  }

  /**
   * Runs the drive based on the Drive Mode and joystick values
   */
  public void Drive() {
    SmartDashboard.putNumber("TOF RANGE",tof.getRange());
    tof.getStatus();
    SmartDashboard.putNumber("TOF light level", tof.getAmbientLightLevel());
    if (!chooserToggle) {
      CurrentDriveMode = DriveModeChooser.getSelected();
    }

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
        m_DriveSubsystem.DriveStateMachine(
            -(x),
            -(y),
            -(z),
            HeldButton,
            HeldButtonReleased,
            ToggleButton,
            ToggleButtonReleased);
        break;

      // two stick driving
      case Two_Stick_Drive:
        x = checkJoystickDeadzone(m_Joystick1.getRawAxis(1), Joystick_Deadzone);
        y = checkJoystickDeadzone(m_Joystick1.getRawAxis(0), Joystick_Deadzone);
        z = checkJoystickDeadzone(m_Joystick2.getRawAxis(0), Joystick_Deadzone);
        angle = (int) DriveSubsystem.getHeading();
        if(Joystick_Deadzone > z && z > -Joystick_Deadzone) {
          correctionPID.calculate(angle , prevAngle);
        } else {
          prevAngle = angle;
        }
        HeldButton = m_Joystick1.getRawButton(13);
        HeldButtonReleased = m_Joystick1.getRawButtonReleased(13);
        m_VisionSubsystem.setLights(1);
        if (m_Joystick1.getRawButtonPressed(4)) {
          m_VisionSubsystem.setLights(3);
          m_VisionSubsystem.resetLimits();
          CurrentDriveMode = DriveModes.Follow_Tape;
          chooserToggle = true;
        }
        ToggleButton = m_Joystick1.getRawButton(12);
        ToggleButtonReleased = m_Joystick1.getRawButtonReleased(12);

        
        roundAngle(angle);
        // System.out.println(angle);

        RotationPoints(m_Joystick2);
        m_DriveSubsystem.DriveStateMachine(
            -(x),
            -(y),
            -(z),
            HeldButton,
            HeldButtonReleased,
            ToggleButton,
            ToggleButtonReleased);
        break;
      case Follow_Tape:
        if (m_Joystick1.getRawButtonPressed(4)) {
          CurrentDriveMode = DriveModes.Two_Stick_Drive;
          chooserToggle = false;
          m_VisionSubsystem.setLights(1);
        }
      if (m_Joystick1.getRawButtonPressed(4)) {
        CurrentDriveMode = DriveModes.Two_Stick_Drive;
        chooserToggle = false;
        m_VisionSubsystem.setLights(1);
      }
        m_VisionSubsystem.switchCamera(m_Joystick1);
        m_VisionSubsystem.updateValues();
        m_VisionSubsystem.followTape();
        
        break;
      // case JoyCon:
      //   // x = checkJoystickDeadzone(m_JoyCons.getRawAxis(1), Joystick_Deadzone);
      //   // y = checkJoystickDeadzone(m_JoyCons.getRawAxis(0), Joystick_Deadzone);
      //   // z = checkJoystickDeadzone(m_JoyCons.getRawAxis(4), Joystick_Deadzone);
      //   // HeldButton = m_JoyCons.getRawButton(13);
      //   // HeldButtonReleased = m_JoyCons.getRawButtonReleased(13);
      //   // ToggleButton = m_JoyCons.getRawButton(12);
      //   // ToggleButtonReleased = m_JoyCons.getRawButtonReleased(12);
      //   // angle = (int) DriveSubsystem.getHeading();
      //   // roundAngle(angle);
      //   // RotationPoints(m_JoyCons);
      //   // m_DriveSubsystem.DriveStateMachine(
      //   //     -(x),
      //   //     -(y),
      //   //     -(z),
      //   //     HeldButton,
      //   //     HeldButtonReleased,
      //   //     ToggleButton,
      //   //     ToggleButtonReleased);
      //   break;
    }

  }

  public void RotationPoints(Joystick m_Joystick2) {
    POVButton pov0 = new POVButton(m_Joystick2, 0); // front
    POVButton pov45 = new POVButton(m_Joystick2, 45); // fr wheel
    POVButton pov90 = new POVButton(m_Joystick2, 90); // right
    POVButton pov135 = new POVButton(m_Joystick2, 135); // br wheel
    POVButton pov180 = new POVButton(m_Joystick2, 180); // back
    POVButton pov225 = new POVButton(m_Joystick2, 225);// bl wheel
    POVButton pov270 = new POVButton(m_Joystick2, 270);// left
    POVButton pov315 = new POVButton(m_Joystick2, 315); // fl wheel
    POVButton povCenter = new POVButton(m_Joystick2, -1);

    // SmartDashboard.putNumber("Limit", Limit);
    if (!(m_Joystick2.getRawAxis(0) <= 0.1 && m_Joystick2.getRawAxis(0) >= -0.1)) {
      if (Limit) {
        Limit = false;
        // SmartDashboard.putNumber("Limit", Limit);
        if (pov0.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Front
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov45.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FR
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov90.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Right
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov135.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BR
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov180.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Back
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov225.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BL
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov270.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Left
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov315.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FL
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else {
          DriveConstants.dCenter = new Translation2d(0, 0);
        }
      } else {
        Limit = false;
        if (povCenter.getAsBoolean()) {
          DriveConstants.dCenter = new Translation2d(0, 0);
        }
      }
    } else {
      Limit = true;
    }
  }

  public void RotationPoints(XboxController m_Joystick2) {
    POVButton pov0 = new POVButton(m_Joystick2, 0); // front
    POVButton pov45 = new POVButton(m_Joystick2, 45); // fr wheel
    POVButton pov90 = new POVButton(m_Joystick2, 90); // right
    POVButton pov135 = new POVButton(m_Joystick2, 135); // br wheel
    POVButton pov180 = new POVButton(m_Joystick2, 180); // back
    POVButton pov225 = new POVButton(m_Joystick2, 225);// bl wheel
    POVButton pov270 = new POVButton(m_Joystick2, 270);// left
    POVButton pov315 = new POVButton(m_Joystick2, 315); // fl wheel
    POVButton povCenter = new POVButton(m_Joystick2, -1);

    // SmartDashboard.putNumber("Limit", Limit);
    if (!(m_Joystick2.getRawAxis(0) <= 0.1 && m_Joystick2.getRawAxis(0) >= -0.1)) {
      if (Limit) {
        Limit = false;
        // SmartDashboard.putNumber("Limit", Limit);
        if (pov0.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Front
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov45.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FR
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov90.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Right
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov135.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BR
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov180.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Back
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov225.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BL
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov270.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Left
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else if (pov315.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FL
              .rotateBy(Rotation2d.fromDegrees(-1 * DriveSubsystem.getHeading()));
        } else {
          DriveConstants.dCenter = new Translation2d(0, 0);
        }
      } else {
        Limit = false;
        if (povCenter.getAsBoolean()) {
          DriveConstants.dCenter = new Translation2d(0, 0);
        }
      }
    } else {
      Limit = true;
    }
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

  public void disableLights() {
    m_VisionSubsystem.setLights(1);
  }

  public DriveModes getDriveState() {
    return CurrentDriveMode;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
