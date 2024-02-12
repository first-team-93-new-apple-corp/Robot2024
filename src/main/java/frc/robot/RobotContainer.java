// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ClimbingLevel;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.TunerConstants;

public class RobotContainer extends TimedRobot {
  public ClimbingLevel m_ClimbingLevel;
  public ShooterCommand m_ShooterCommand;
  public ClimberCommand m_ClimberCommand;
  public IntakeCommand m_IntakeCommand;
  public ElevatorCommand m_ElevatorCommand;
  private SwerveRequest.ApplyChassisSpeeds m_swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
  private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SendableChooser<Command> autoChooser;
  public final double MaxSpeed = DriveConstants.MaxSpeed;
  public final double MaxAngularRate = DriveConstants.MaxAngularRate;
  private double angle;
  private double deadzone = DriveConstants.JoystickDeadzone;
  private Joystick m_Joystick1;
  private Joystick m_Joystick2;
  private XboxController op;
  private boolean Limit = true;

  // can set this to whatever button you want you can also just
  // delete this and use the constants file for the button
  // (just so that the logic works for now)
  private int climbingLevelButton;

  private ChassisSpeeds speeds;
  private ChassisSpeeds fieldSpeeds;
  private double fieldRelativeOffset;
  private final JoystickButton m_JoystickTrigger;
  private final JoystickButton m_JoystickButton2;
  private final JoystickButton m_RobotRelButton;

  private final JoystickButton m_CameraRelButton;
  final Field2d m_field = new Field2d();

  // added this for button bindings and the logic I added
  private final JoystickButton m_climingLevelButton;

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
    m_Joystick2 = this.m_Joystick2;
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
          DriveConstants.dCenter = DriveConstants.Center;
        }
      } else {
        Limit = false;
        if (povCenter.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Center;
        }
      }
    } else {
      Limit = true;
    }
  }

  public void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> m_swerveRequest
            .withCenterOfRotation(DriveConstants.dCenter)
            .withSpeeds(fieldSpeeds)));

    // Brake while held
    m_JoystickTrigger.whileTrue(drivetrain.applyRequest(() -> brake));
    m_RobotRelButton.onTrue(drivetrain.applyRequest(() -> RobotCentricDrive
        .withVelocityX(
            -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)
                * MaxSpeed)
        .withVelocityY(
            -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxSpeed)
        .withRotationalRate(
            -m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxAngularRate)));

    m_CameraRelButton.whileTrue(drivetrain.applyRequest(() -> RobotCentricDrive
        .withVelocityX(
            m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)
                * MaxSpeed)
        .withVelocityY(
            m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxSpeed)
        .withRotationalRate(
            -m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxAngularRate)));

    // Climbing Level logic added to button
    // When the button of your choosing is held it should atomaticly do the climbing
    // level
    
    // m_climingLevelButton.whileTrue(m_ClimbingLevel);

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

  public void configAuto() {
    drivetrain.configAuto();
  }

  public RobotContainer(Joystick m_Joystick1, Joystick m_Joystick2, XboxController op) {
    m_JoystickTrigger = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Trigger);
    m_JoystickButton2 = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Center_Button);
    m_RobotRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Left_Buttons.Bottom_Middle);
    m_CameraRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Trigger);
    m_climingLevelButton = new JoystickButton(op, climbingLevelButton);
    SmartDashboard.putData("Field", m_field);
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;
    this.op = op;
    IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    NamedCommands.registerCommand("Intake", m_IntakeSubsystem.AutoIntake());
    NamedCommands.registerCommand("PrimeShooter", m_ShooterSubsystem.AutonShooter());
    NamedCommands.registerCommand("Shooter", m_ShooterSubsystem.AutonKicker());

    drivetrain.configAuto();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void updateValues() {
    SmartDashboard.putNumber("PigeonAngle", angle);
    m_field.setRobotPose(drivetrain.getPose());
    if (m_Joystick1.getRawButtonPressed(Constants.Thrustmaster.Left_Buttons.Top_Middle)) {
      fieldRelativeOffset = drivetrain.getPigeon2().getRotation2d().getRadians();
    }
    speeds = new ChassisSpeeds(
        (checkDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)) * MaxSpeed),
        (checkDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxSpeed),
        (checkDeadzone(-m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxAngularRate));
    fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
        new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
            .rotateBy(new Rotation2d(-fieldRelativeOffset)));
    SmartDashboard.putNumber("E", new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
        .rotateBy(new Rotation2d(-fieldRelativeOffset)).getDegrees());
    RotationPoints(m_Joystick2);
    POVButton();
  }

  public double checkDeadzone(double input) {
    if (deadzone > input && input > -deadzone) {
      return 0;
    } else {
      return input;
    }
  }

  public SwerveDriveSubsystem getDrive() {
    return drivetrain;
  }

  public Pigeon2 getPigeon() {
    return drivetrain.getPigeon2();
  }
}