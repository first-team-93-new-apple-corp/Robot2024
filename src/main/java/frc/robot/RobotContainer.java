// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ARM_SETPOINTS;
import frc.robot.commands.ArmToSetpoint;
import frc.robot.commands.ElevatorZeroCommand;
import frc.robot.commands.NoteHandle;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
// import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.TunerConstants;
import frc.robot.subsystems.Helpers.ArmCalculation;
import frc.robot.subsystems.Helpers.ArmHelper;
import frc.robot.subsystems.Helpers.LimelightHelpers;
import frc.robot.subsystems.Helpers.Vision;

public class RobotContainer {
  // Constants / Other things
  private ChassisSpeeds speeds;
  private ChassisSpeeds fieldSpeeds;
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps / 1.25; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = (1.5 * Math.PI) / 1.25; // 3/4 of a rotation per second max angular velocity
  private SwerveRequest.ApplyChassisSpeeds m_swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
  // Joysticks / Controllers
  // private final Joystick m_LeftStick = new Joystick(0);
  // private final Joystick m_RightStick = new Joystick(1);
  private final CommandXboxController m_XboxDriver = new CommandXboxController(0);
  // Joystick Buttons
  // private final JoystickButton m_FieldRelativeButton = new
  // JoystickButton(m_LeftStick, 12);
  // Drivetrain
  private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private SendableChooser<Command> autoChooser;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  // private final SwerveRequest.RobotCentric robotDrive = new
  // SwerveRequest.RobotCentric()
  // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) //
  // Add a 10% deadband
  // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // private final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
  // private final SwerveRequest.PointWheelsAt point = new
  // SwerveRequest.PointWheelsAt();
  private ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private double FieldRelativeOffset = 0;
  private ArmHelper m_ArmHelper;
  private ShoulderSubsystem m_ShoulderSubsystem = new ShoulderSubsystem();
  private ArmCalculation m_ArmCalculation = new ArmCalculation(m_ShoulderSubsystem, m_ShooterSubsystem,() -> drivetrain.getpPose2d());
  private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private Vision m_Vision = new Vision(drivetrain.getPigeon2());
  private ElevatorZeroCommand m_ElevatorZeroCommand = new ElevatorZeroCommand(m_ElevatorSubsystem);
  private IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private NoteHandle noteHandle = new NoteHandle(m_IntakeSubsystem, m_ShooterSubsystem);
  private double lastSet = 0;
  private void configureBindings() {
    SmartDashboard.putNumber("Shoulder Test Setpoint", 0);
    m_Vision.periodic();
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> m_swerveRequest
            .withCenterOfRotation(DriveConstants.Center)
            .withSpeeds(fieldSpeeds)));
    m_XboxDriver.leftBumper().onTrue((drivetrain
        .applyRequest(() -> m_swerveRequest.withCenterOfRotation(DriveConstants.Center).withSpeeds(fieldSpeeds))));
    m_XboxDriver.leftStick().whileTrue(rotateToNote());
    // Subsystems
    // m_ShoulderSubsystem.init();
    m_ElevatorSubsystem.init();
    m_ArmHelper = new ArmHelper(m_ShoulderSubsystem, m_ElevatorSubsystem);

    m_XboxDriver.y().whileTrue(new ArmToSetpoint(m_ArmHelper,
        ARM_SETPOINTS.Amp));
    m_XboxDriver.b().whileTrue(new ArmToSetpoint(m_ArmHelper, ARM_SETPOINTS.Intake));
    m_XboxDriver.rightBumper().whileTrue(Commands.run(() -> noteHandle.demoIntake()));
    m_XboxDriver.rightBumper().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    m_XboxDriver.x().whileTrue(Commands.run(() -> noteHandle.intake()));
    m_XboxDriver.a().onTrue(Commands.runOnce(() -> m_ShoulderSubsystem.demo()));
    m_XboxDriver.rightTrigger().whileTrue(Commands.run(() -> noteHandle.shoot()));
    m_XboxDriver.rightTrigger().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    m_XboxDriver.x().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    m_XboxDriver.leftTrigger().whileTrue(Commands.run(() -> noteHandle.prime()));
    m_XboxDriver.leftTrigger().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    m_XboxDriver.povUp().onTrue(Commands.runOnce(() -> m_ShoulderSubsystem.testup()));
    m_XboxDriver.povDown().onTrue(Commands.runOnce(() -> m_ShoulderSubsystem.testdown()));
    m_XboxDriver.povLeft().whileTrue(Commands.run(() -> noteHandle.revShoot()));
    m_XboxDriver.povLeft().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    m_XboxDriver.leftBumper().whileTrue(Commands.run(() -> noteHandle.amp()));
    m_XboxDriver.leftBumper().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    drivetrain.setDefaultCommand( // Drivetrain will execute this commandperiodically
        drivetrain
            .applyRequest(
                () -> drive.withVelocityX((-m_XboxDriver.getLeftY() *
                    Math.abs(m_XboxDriver.getLeftY())) * MaxSpeed)
                    .withVelocityY((-m_XboxDriver.getLeftX() * Math.abs(m_XboxDriver.getLeftX()))
                        * MaxSpeed)
                    .withRotationalRate(
                        (-m_XboxDriver.getRightX() * Math.abs(m_XboxDriver.getRightX())
                            * Math.abs(m_XboxDriver.getRightX()))
                            * MaxAngularRate)));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    NamedCommands.registerCommand("Stop", Commands.runOnce(() -> noteHandle.stop()));
    NamedCommands.registerCommand("Ready", Commands.runOnce(() -> noteHandle.intake()));
    NamedCommands.registerCommand("Fire", Commands.runOnce(() -> noteHandle.shoot()));
    NamedCommands.registerCommand("Aim", Commands.runOnce(() -> noteHandle.revShoot()));
    NamedCommands.registerCommand("Kick", Commands.runOnce(() -> noteHandle.prime()));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public RobotContainer() {
    configureBindings();
  }

  public void updateValues() {
    rotateToNote();
    // if (m_LeftStick.getRawButton(Constants.Thrustmaster.Left_Buttons.Top_Middle))
    // {
    // FieldRelativeOffset = drivetrain.getPigeon2().getRotation2d().getRadians();
    // drivetrain.getPigeon2().reset();
    // }
    if (m_XboxDriver.leftBumper().getAsBoolean()) {
      FieldRelativeOffset = drivetrain.getPigeon2().getRotation2d().getRadians();
      drivetrain.getPigeon2().reset();
    }
    speeds = new ChassisSpeeds(
        (checkDeadzone(-m_XboxDriver.getLeftY() * MaxSpeed)),
        (checkDeadzone(-m_XboxDriver.getLeftX() * MaxSpeed)),
        (checkDeadzone(-m_XboxDriver.getRightX() * MaxAngularRate)));
    fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
        new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians()));
    SmartDashboard.putNumber("Shoulder Angle",
        m_ShoulderSubsystem.getPosition());
    SmartDashboard.putBoolean("Shoulder At Setpoint",
        m_ShoulderSubsystem.atSetpoint());
    SmartDashboard.putBoolean("Elevator Hall Effect",
        m_ElevatorSubsystem.getZero());
    SmartDashboard.putNumber("Elevator Position", m_ElevatorSubsystem.getPosition());
    SmartDashboard.putNumber("Shoulder L", m_ShoulderSubsystem.getLeft());
    SmartDashboard.putNumber("Shoulder R", m_ShoulderSubsystem.getRight());
    SmartDashboard.putBoolean("Has Note", noteHandle.hasNote());
    SmartDashboard.putNumber("Rotate Note", m_Vision.turnToNote());
    if (lastSet != SmartDashboard.getNumber("Shoulder Test Setpoint", 0)) {
      lastSet = SmartDashboard.getNumber("Shoulder Test Setpoint", 0);
      m_ShoulderSubsystem.toSetpoint(lastSet);
    }
    drivetrain.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue("limelight-front"), Timer.getFPGATimestamp());
  }

  public Command getTeleopCommand() {
    return null;
  }

  public Command rotateToNote() {
    return drivetrain.applyRequest(() -> drive.withRotationalRate(m_Vision.turnToNote()));
  }

  public void checkZero() {
    m_ElevatorSubsystem.checkZero();
  }

  public void zeroElevator() {
    m_ElevatorZeroCommand.schedule();
  }

  public void endzeroElevator() {
    m_ElevatorZeroCommand.cancel();
  }

  public double checkDeadzone(double input) {
    if (0.1 > input && input > -0.1) {
      return 0;
    } else {
      return input;
    }
  }
}
