// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ARM_SETPOINTS;
import frc.robot.commands.ArmToSetpoint;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
// import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.TunerConstants;
import frc.robot.subsystems.Helpers.ArmHelper;
import frc.robot.subsystems.IntakeShooterSubsystem;

import frc.robot.commands.ElevatorZeroCommand;

public class RobotContainer {
  // Constants / Other things
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // Joysticks / Controllers
  private final Joystick m_LeftStick = new Joystick(0);
  private final Joystick m_RightStick = new Joystick(1);
  private final XboxController m_XboxDriver = new XboxController(0);

  // Joystick Buttons
  private final JoystickButton m_FieldRelativeButton = new JoystickButton(m_LeftStick, 12);
  private final JoystickButton m_FieldRelativeButtonXbox = new JoystickButton(m_XboxDriver, 5);
  private final JoystickButton m_AmpButtonXbox = new JoystickButton(m_XboxDriver, Constants.xbox.Y);
  private final JoystickButton m_IntakeButtonXbox = new JoystickButton(m_XboxDriver, Constants.xbox.X);
  private final JoystickButton m_IntakeNoteXbox = new JoystickButton(m_XboxDriver, Constants.xbox.B);
  private final JoystickButton TestRunElevator = new JoystickButton(m_XboxDriver, Constants.xbox.RightShoulderButton);
  private final JoystickButton TestZeroElevator = new JoystickButton(m_XboxDriver, Constants.xbox.A);
  
  // private final JoystickButton m_AimButtonXbox = new JoystickButton(m_XboxDriver, Constants.xbox.A);
  // private final JoystickButton m_FireButtonXbox = new
  // JoystickButton(m_XboxDriver, Constants.xbox.B);
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
  private ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private IntakeShooterSubsystem m_IntakeShooterSubsystem = new IntakeShooterSubsystem();

  ElevatorZeroCommand m_ElevatorZeroCommand = new ElevatorZeroCommand(m_ElevatorSubsystem);

  private void configureBindings() {

    // SmartDashboard.putNumber("Elevator Setpoint", 35);

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
    m_FieldRelativeButtonXbox.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Subsystems
    m_ShoulderSubsystem.init();
    m_ElevatorSubsystem.init();
    m_IntakeShooterSubsystem.init();
    m_ArmHelper = new ArmHelper(m_ShoulderSubsystem, m_ElevatorSubsystem);

    // // The funny buttons
    //X
    m_AmpButtonXbox.whileTrue(new ArmToSetpoint(m_ArmHelper, ARM_SETPOINTS.Amp));
    //Y
    m_IntakeButtonXbox.whileTrue(new ArmToSetpoint(m_ArmHelper, ARM_SETPOINTS.Intake));
    //B
    m_IntakeNoteXbox.whileTrue(m_IntakeShooterSubsystem.AutoIntake());
    m_IntakeNoteXbox.whileFalse(m_IntakeShooterSubsystem.stop());

    // TestRunElevator.whileTrue(m_ElevatorSubsystem.toSetpoint(15));
    // TestRunElevator.whileFalse(m_ElevatorSubsystem.lock());
    // TestZeroElevator.whileTrue(m_ElevatorZeroCommand);


    // // for auto aim (later)
    // m_AimButtonXbox.whileTrue(new ArmToSetpoint(m_ArmHelper,
    // ARM_SETPOINTS.Shoot));

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

  }

  public Command getTeleopCommand() {
    return null;
  }
  public void checkZero() {
    m_ElevatorSubsystem.checkZero();
  }

}
