// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.TunerConstants;

public class RobotContainer {
  public final double MaxSpeed = DriveConstants.MaxSpeed;
  public final double MaxAngularRate = DriveConstants.MaxAngularRate;
  private final Joystick m_Joystick1 = new Joystick(0);
  private final Joystick m_Joystick2 = new Joystick(1);
  private final JoystickButton m_JoystickTrigger = new JoystickButton(m_Joystick1, 1);
  private final JoystickButton m_fieldRelButton = new
  JoystickButton(m_Joystick1,
  Constants.Thrustmaster.Left_Buttons.Top_Middle);
  private final JoystickButton m_JoystickButton2 = new
  JoystickButton(m_Joystick1, 2);
  private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Configures the bindings to drive / control the swerve drive :)
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(
                    -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)
                    * MaxSpeed)
            .withVelocityY(
                    -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)
                    * MaxSpeed)
            .withRotationalRate(
                    -m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)
                    * MaxAngularRate)));
    // Brake while held
    m_JoystickTrigger.onTrue(drivetrain.applyRequest(() -> brake));
    m_fieldRelButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
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
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getTeleopCommand() {
    return null;
  }
}
