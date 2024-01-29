// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.TunerConstants;

public class RobotContainer extends TimedRobot {
  private SwerveRequest.ApplyChassisSpeeds m_swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
  private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SendableChooser<Command> autoChooser;
  public final double MaxSpeed = DriveConstants.MaxSpeed;
  public final double MaxAngularRate = DriveConstants.MaxAngularRate;
  private final Joystick m_Joystick1 = new Joystick(0);
  private final Joystick m_Joystick2 = new Joystick(1);
  private Pigeon2 m_Pigeon2 = new Pigeon2(0);
  private boolean Limit = true;
  private ChassisSpeeds speeds;
  private ChassisSpeeds fieldSpeeds;
  private double fieldRelativeOffset;
  private final JoystickButton m_JoystickTrigger = new JoystickButton(m_Joystick1, 1);
  private final JoystickButton m_fieldRelButton = new JoystickButton(m_Joystick1,
      Constants.Thrustmaster.Left_Buttons.Top_Middle);
  private final JoystickButton m_JoystickButton2 = new JoystickButton(m_Joystick1, 2);

  private final JoystickButton m_RobotRelButton = new JoystickButton(m_Joystick1,
      Constants.Thrustmaster.Left_Buttons.Bottom_Middle);
  private SwerveRequest.FieldCentric FieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private SwerveRequest.RobotCentric RobotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  POVButton pov0; // front
  POVButton pov45; // fr wheel
  POVButton pov90; // right
  POVButton pov135; // br wheel
  POVButton pov180; // back
  POVButton pov225;// bl wheel
  POVButton pov270;// left
  POVButton pov315; // fl wheel
  POVButton povCenter;

  // Configures the bindings to drive / control the swerve drive :)
  public void RotationPoints(Joystick m_Joystick2) {
    pov0 = new POVButton(m_Joystick2, 0); // front
    pov45 = new POVButton(m_Joystick2, 45); // fr wheel
    pov90 = new POVButton(m_Joystick2, 90); // right
    pov135 = new POVButton(m_Joystick2, 135); // br wheel
    pov180 = new POVButton(m_Joystick2, 180); // back
    pov225 = new POVButton(m_Joystick2, 225);// bl wheel
    pov270 = new POVButton(m_Joystick2, 270);// left
    pov315 = new POVButton(m_Joystick2, 315); // fl wheel
    povCenter = new POVButton(m_Joystick2, -1);
  }

  public void POVButton() {
    // SmartDashboard.putNumber("Limit", Limit);
    if (!(m_Joystick2.getRawAxis(0) <= 0.1 && m_Joystick2.getRawAxis(0) >= -0.1)) {
      if (Limit) {
        Limit = false;
        // SmartDashboard.putNumber("Limit", Limit);
        if (pov0.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Front
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading()));
        } else if (pov45.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FR
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading()));
        } else if (pov90.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Right
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading()));
        } else if (pov135.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BR
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading()));
        } else if (pov180.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Back
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading()));
        } else if (pov225.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BL
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading()));
        } else if (pov270.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Left
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading()));
        } else if (pov315.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FL
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading()));
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

  public void configureBindings() {
    RotationPoints(m_Joystick2);
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
        FieldCentricDrive
        .withVelocityX(
        -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)
        * MaxSpeed)
        .withVelocityY(
        -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)
        * MaxSpeed)
        .withRotationalRate(
        -m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)
        * MaxAngularRate)
        // m_swerveRequest
        //     .withCenterOfRotation(DriveConstants.dCenter)
        //     .withSpeeds(fieldSpeeds)
            ));

    // Brake while held
    m_JoystickTrigger.whileTrue(drivetrain.applyRequest(() -> brake));
    m_fieldRelButton.onTrue(drivetrain.runOnce(() ->
    drivetrain.seedFieldRelative()));


    // Points all in a direction
    m_JoystickButton2.whileTrue(drivetrain
        .applyRequest(
            () -> point.withModuleDirection(new Rotation2d(-m_Joystick1.getRawAxis(0),
                -m_Joystick1.getRawAxis(1)))));

    // reset the field-centric heading on left bumper press

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    drivetrain.configAuto();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  public Command getAutonomousCommand() {
    return drivetrain.getAutoPath("Example Path");
  }

  public void updateValues() {
    for(double angle = m_Pigeon2.getAngle(); angle%360 > 1;) {
      angle -= 360;
    }
    if(m_Joystick1.getRawButton(Constants.Thrustmaster.Left_Buttons.Top_Middle)) {
      fieldRelativeOffset = m_Pigeon2.getAngle();
    }
    speeds = new ChassisSpeeds(
        (-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y) * MaxSpeed),
        (-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x) * MaxSpeed),
        (-m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x) * MaxAngularRate));
    // fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, new Rotation2d( m_Pigeon2.getAngle()%360));
    fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_Pigeon2.getRotation2d().minus(new Rotation2d(fieldRelativeOffset)));
    SmartDashboard.putNumber("Pigeon Rot2d", m_Pigeon2.getAngle()%360);
    // fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds((-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y) * MaxSpeed), (-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x) * MaxSpeed), (-m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x) * MaxAngularRate), drivetrain.getPigeon2().getRotation2d());
    POVButton();
  }
}