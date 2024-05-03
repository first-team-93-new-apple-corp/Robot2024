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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeShooterCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.TunerConstants;

public class RobotContainer {
  // Constants / Other things
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private IntakeShooterCommand m_IntakeShooter;

  // Joysticks / Controllers
  private final Joystick m_LeftStick = new Joystick(0);
  private final Joystick m_RightStick = new Joystick(1);
  private final XboxController m_XboxDriver = new XboxController(0);

  // Joystick Buttons
  private final JoystickButton m_FieldRelativeButton = new JoystickButton(m_LeftStick, 12);
  private final JoystickButton m_FieldRelativeButtonXbox = new JoystickButton(m_XboxDriver, 5);

  // Drivetrain
  private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private SendableChooser<Command> autoChooser;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    // Controls
    // reset the field-centric heading on left bumper press
    m_FieldRelativeButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    m_FieldRelativeButtonXbox.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
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

    NamedCommands.registerCommand("Intake", m_IntakeShooter.Intake());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public RobotContainer() {
    configureBindings();
  }

}
