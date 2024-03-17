// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystemFactory;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystemFactory;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystemFactory;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystemFactory;
import frc.robot.subsystems.Swerve.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystemFactory;
import frc.robot.subsystems.Swerve.Telemetry;

public class RobotContainer extends TimedRobot {

  // public ClimbingLevel m_ClimbingLevel;
  public ShooterCommand m_ShooterCommand;
  public IntakeCommand m_IntakeCommand;
  public ElevatorCommand m_ElevatorCommand;  
  public ShooterSubsystem m_ShooterSubsystem;
  public IntakeSubsystem m_IntakeSubsystem;
  public ElevatorSubsystem m_ElevatorSubsystem;
  public ClimberSubsystem m_ClimberSubsystem;
  public Mechanisms m_Mechanisms;
  private SwerveRequest.ApplyChassisSpeeds m_swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
  private final SwerveDriveSubsystem drivetrain; // My drivetrain
  private SendableChooser<Command> autoChooser;
  public final double MaxSpeed = DriveConstants.MaxSpeed;
  public final double MaxAngularRate = DriveConstants.MaxAngularRate;
  private double angle;
  private double deadzone = DriveConstants.JoystickDeadzone;
  private Joystick m_Joystick1;
  private Joystick m_Joystick2;
  @SuppressWarnings("unused")
  private XboxController op;
  private boolean Limit = true;
  // can set this to whatever button you want you can also just
  // delete this and use the constants file for the button
  // (just so that the logic works for now)
  // private int climbingLevelButton;

  private ChassisSpeeds speeds;
  private ChassisSpeeds fieldSpeeds;
  private double fieldRelativeOffset;
  JoystickButton m_AmpAlignButton;
  JoystickButton m_TrapAlignButton;
  private final JoystickButton m_BrakeButton;
  private final JoystickButton m_fieldRelButton;
  private final JoystickButton m_RobotRelButton;

  private final JoystickButton m_CameraRelButton;
  public Field2d m_Field2d = new Field2d();
  private Pose2d pose = new Pose2d();
  // private final SwerveDrivePoseEstimator m_poseEstimator;
  // added this for button bindings and the logic I added
  
  AutoAlignCommand m_AutoAlignCommand;

  private SwerveRequest.RobotCentric RobotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

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

  // logan was here you silly gooses ;)
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
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov45.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FR
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov90.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Right
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov135.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BR
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov180.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Back
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov225.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BL
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov270.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Left
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov315.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FL
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
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
    m_TrapAlignButton.whileTrue(m_AutoAlignCommand);
    m_AmpAlignButton.whileTrue(m_AutoAlignCommand);
    
    // m_AmpAlignButton.whileTrue(drivetrain.applyRequest(() -> m_swerveRequest
    // .withCenterOfRotation(DriveConstants.dCenter)
    // .withSpeeds(m_AutoAlignSubsystem.fieldSpeeds)));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> m_swerveRequest
            .withCenterOfRotation(DriveConstants.dCenter)
            .withSpeeds(fieldSpeeds)));

    m_fieldRelButton.onTrue(
        drivetrain.applyRequest(() -> m_swerveRequest
            .withCenterOfRotation(DriveConstants.dCenter)
            .withSpeeds(fieldSpeeds)));

    // Brake while held
    m_BrakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
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
    // m_climbingLevelButton.whileTrue(m_ClimbingLevel);

    // Points all in a direction
    // m_WheelsPointForwardButton.whileTrue(drivetrain
    //     .applyRequest(
    //         () -> point.withModuleDirection(new Rotation2d(-m_Joystick1.getRawAxis(0),
    //             -m_Joystick1.getRawAxis(1)))));

    // reset the field-centric heading on left bumper press

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void configAuto() {
    drivetrain.configAuto();
  }
  public Pose3d getTagPose(AprilTag tag){
    return tag.pose;
  }

  public RobotContainer(Constants constants, Joystick m_Joystick1, Joystick m_Joystick2, XboxController op, LEDSubsystem m_LedSubsystem) {
    this.m_ShooterSubsystem = ShooterSubsystemFactory.build(constants.Shooter);
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;
    this.op = op;
    drivetrain = SwerveDriveSubsystemFactory.build(constants.Drive);
    m_IntakeSubsystem = IntakeSubsystemFactory.build(constants.Intake,m_LedSubsystem, m_ShooterSubsystem, op);
    m_ElevatorSubsystem = ElevatorSubsystemFactory.build(constants.Elevator);
    m_ClimberSubsystem = ClimberSubsystemFactory.build(constants.Climber);

    m_IntakeCommand = new IntakeCommand(m_ShooterSubsystem, m_IntakeSubsystem, m_LedSubsystem);
    m_ShooterCommand = new ShooterCommand(m_ShooterSubsystem);


    SmartDashboard.putData("Field",m_Field2d);

    m_Mechanisms = new Mechanisms(m_ElevatorSubsystem, m_ClimberSubsystem);

    m_AutoAlignCommand = new AutoAlignCommand(drivetrain, m_Joystick1);
    m_fieldRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Left_Buttons.Top_Middle);
    m_BrakeButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Trigger);
    // m_WheelsPointForwardButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Center_Button);
    m_RobotRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Left_Buttons.Bottom_Middle);
    m_CameraRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Trigger);
    m_AmpAlignButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Center_Button);
    m_TrapAlignButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Button);
    //ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    //IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(m_ShooterSubsystem);
    // m_AutoAlignSubsystem = new AutoAlignSubsystem(drivetrain);
    // m_climbingLevelButton = new JoystickButton(op, climbingLevelButton);
    NamedCommands.registerCommand("Intake", m_IntakeCommand.AutoIntake());
    NamedCommands.registerCommand("Shooter", m_ShooterCommand.AutonShooter());
    NamedCommands.registerCommand("StopShooter", m_ShooterCommand.AutonStopShooter());
    NamedCommands.registerCommand("ShootAmp", m_ShooterCommand.AutonAmp());
    NamedCommands.registerCommand("DribbleNote", m_ShooterCommand.AutonDribbleNote());
    // NamedCommands.registerCommand("StopKicker", m_ShooterSubsystem.AutonKickerStop());
    NamedCommands.registerCommand("StopIntake", m_IntakeCommand.AutonStopIntake());
    NamedCommands.registerCommand("ResetField", drivetrain.resetPigeonAuton());
    SignalLogger.start();
    drivetrain.configAuto();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public void updateVision() {
    drivetrain.updateOdometry();
    pose = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition();
    m_Field2d.setRobotPose(pose);
  }
  public void updateValues() {
    m_Mechanisms.periodic();
    angle = drivetrain.getHeading();
    SmartDashboard.putNumber("PigeonAngle", angle);
    if (m_Joystick1.getRawButtonPressed(Constants.Thrustmaster.Left_Buttons.Top_Middle)) {
      fieldRelativeOffset = drivetrain.getPigeon2().getRotation2d().getRadians();
      drivetrain.getPigeon2().reset();
    }
    speeds = new ChassisSpeeds(
        (checkDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)) * MaxSpeed),
        (checkDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxSpeed),
        (checkDeadzone(-m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxAngularRate));
    fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
        new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
            // .rotateBy(new Rotation2d(-fieldRelativeOffset))
            );
    RotationPoints(m_Joystick2);
    POVButton();
    // m_AutoAlignSubsystem.Alliance();
    updateVision();
    SignalLogger.writeDoubleArray("pose", new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
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
