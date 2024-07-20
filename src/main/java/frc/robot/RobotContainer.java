// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ARM_SETPOINTS;
import frc.robot.commands.ArmToSetpoint;
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
import frc.robot.subsystems.Helpers.Vision;

import frc.robot.commands.ElevatorZeroCommand;
import frc.robot.commands.NoteHandle;

public class RobotContainer {
  // Constants / Other things
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // Joysticks / Controllers
  private final Joystick m_LeftStick = new Joystick(0);
  private final Joystick m_RightStick = new Joystick(1);
  private final CommandXboxController m_XboxDriver = new CommandXboxController(2);

  // Joystick Buttons
  private final JoystickButton m_FieldRelativeButton = new JoystickButton(m_LeftStick, 12);
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
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private ArmHelper m_ArmHelper;
  private ShoulderSubsystem m_ShoulderSubsystem = new ShoulderSubsystem();
  private ArmCalculation m_ArmCalculation = new ArmCalculation(m_ShoulderSubsystem);
  private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private Vision m_Vision = new Vision();
  private ElevatorZeroCommand m_ElevatorZeroCommand = new ElevatorZeroCommand(m_ElevatorSubsystem);
  private IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private NoteHandle noteHandle = new NoteHandle(m_IntakeSubsystem, m_ShooterSubsystem);

  private void configureBindings() {
    // SmartDashboard.putNumber("Elevator Setpoint", 35);
    m_Vision.periodic();
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(
                -m_LeftStick.getRawAxis(Constants.Thrustmaster.Axis.y)
                    * MaxSpeed)
            .withVelocityY(
                -m_LeftStick.getRawAxis(Constants.Thrustmaster.Axis.x)
                    * MaxSpeed)
            .withRotationalRate(
                -m_RightStick.getRawAxis(Constants.Thrustmaster.Axis.x)
                    * MaxAngularRate)));
    // Brake while held
    // m_JoystickTrigger.onTrue(drivetrain.applyRequest(() -> brake));
    // Points all in a direction

    // reset the field-centric heading on left bumper press
    m_FieldRelativeButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // m_XboxDriver.leftBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldRelative()));

    // Subsystems
    // m_ShoulderSubsystem.init();
    m_ElevatorSubsystem.init();
    m_ArmHelper = new ArmHelper(m_ShoulderSubsystem, m_ElevatorSubsystem);

    // // The funny buttons
    // Y
    m_XboxDriver.y().whileTrue(new ArmToSetpoint(m_ArmHelper, ARM_SETPOINTS.Amp));
    // B
    m_XboxDriver.b().whileTrue(new ArmToSetpoint(m_ArmHelper, ARM_SETPOINTS.Intake));
    // X
    m_XboxDriver.x().whileTrue(Commands.run(() -> noteHandle.intake()));
    // Right trigger
    m_XboxDriver.rightTrigger().whileTrue(Commands.run(() -> noteHandle.shoot()));
    // m_XboxDriver.rightTrigger().onFalse(Commands.runOnce(() ->
    // checkConflictShoot()));
    m_XboxDriver.rightTrigger().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    // m_XboxDriver.x().onFalse(Commands.runOnce(() -> checkConflictIntake()));
    m_XboxDriver.x().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    // m_XboxDriver.leftTrigger().whileTrue(new ArmToSetpoint(m_ArmHelper,
    // ARM_SETPOINTS.Shoot));
    // left trigger
    m_XboxDriver.leftTrigger().whileTrue(Commands.run(() -> noteHandle.prime()));
    m_XboxDriver.leftTrigger().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    // Pov Buttons
    m_XboxDriver.povUp().onTrue(Commands.runOnce(() -> m_ShoulderSubsystem.testup()));
    m_XboxDriver.povDown().onTrue(Commands.runOnce(() -> m_ShoulderSubsystem.testdown()));
    m_XboxDriver.povLeft().whileTrue(Commands.run(() -> noteHandle.revShoot()));
    m_XboxDriver.povLeft().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    m_XboxDriver.leftBumper().whileTrue(Commands.run(() -> noteHandle.amp()));
    m_XboxDriver.leftBumper().onFalse(Commands.runOnce(() -> noteHandle.stop()));
    // Funny a button
    m_XboxDriver.a().whileTrue(
        drivetrain.applyRequest(() -> drive
            .withVelocityX((-m_XboxDriver.getLeftY() * Math.abs(m_XboxDriver.getLeftY())) * MaxSpeed)
            .withVelocityY((-m_XboxDriver.getLeftX() * Math.abs(m_XboxDriver.getLeftX())) * MaxSpeed)
            .withRotationalRate(m_Vision.pointToCalc())));
    // Drive Controls
    if (DriverStation.getJoystickIsXbox(0)) {
      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          drivetrain
              .applyRequest(
                  () -> drive.withVelocityX((-m_XboxDriver.getLeftY() * Math.abs(m_XboxDriver.getLeftY())) * MaxSpeed)
                      .withVelocityY((-m_XboxDriver.getLeftX() * Math.abs(m_XboxDriver.getLeftX())) * MaxSpeed)
                      .withRotationalRate(
                          (-m_XboxDriver.getRightX() * Math.abs(m_XboxDriver.getRightX())
                              * Math.abs(m_XboxDriver.getRightX()))
                              * MaxAngularRate)));
    } else {
      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() -> drive.withVelocityX(-m_LeftStick.getRawAxis(1) * MaxSpeed) // Drive forward with
              // negative Y (forward)
              .withVelocityY(-m_LeftStick.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-m_RightStick.getRawAxis(0) * MaxAngularRate) // Drive counterclockwise with negative
                                                                                // X (left)
          ));
    }

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    NamedCommands.registerCommand("StopShootIntake", Commands.runOnce(() -> noteHandle.stop()));
    NamedCommands.registerCommand("Ready", Commands.runOnce(() -> noteHandle.intake()));
    NamedCommands.registerCommand("Fire", Commands.runOnce(() -> noteHandle.shoot()));
    NamedCommands.registerCommand("Aim", Commands.runOnce(() -> noteHandle.revShoot())
    .andThen(Commands.waitSeconds(.75))
    .andThen(Commands.runOnce(() -> noteHandle.stop())));
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

  }

  public Command getTeleopCommand() {
    return null;
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

  // public void checkConflictShoot() {
  //   if (!m_XboxDriver.x().getAsBoolean()) {
  //     noteHandle.stop();
  //   }
  // }

  // public void checkConflictIntake() {
  //   if (!m_XboxDriver.rightTrigger().getAsBoolean()) {
  //     noteHandle.stop();
  //   }
  // }

  // public void checkConflict() {
  //   if (!(m_XboxDriver.leftBumper().getAsBoolean() && m_XboxDriver.povLeft().getAsBoolean()
  //       && m_XboxDriver.rightTrigger().getAsBoolean() && m_XboxDriver.leftTrigger().getAsBoolean())) {
  //         noteHandle.stop();
  //   }
  // }

}
