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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final Joystick m_LeftStick = new Joystick(0);
  private final Joystick m_RightStick = new Joystick(1);
  private final XboxController m_Driver = new XboxController(0);
  private final JoystickButton m_FieldRelativeButton = new JoystickButton(m_LeftStick, 12);
  private final JoystickButton m_FieldRelativeButtonXbox = new JoystickButton(m_Driver, 5);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    if(DriverStation.getJoystickIsXbox(0)) {
      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> drive.withVelocityX((-m_Driver.getLeftY() * Math.abs(m_Driver.getLeftY())) * MaxSpeed) // Drive forward with
                                                                                         // negative Y (forward)
          .withVelocityY((-m_Driver.getLeftX() * Math.abs(m_Driver.getLeftX())) * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate((-m_Driver.getRightX() * Math.abs(m_Driver.getRightX())) * MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));
    } else {
      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> drive.withVelocityX(-m_LeftStick.getRawAxis(1) * MaxSpeed) // Drive forward with
                                                                                         // negative Y (forward)
          .withVelocityY(-m_LeftStick.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-m_RightStick.getRawAxis(0)* MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));
    }
    

   

    // reset the field-centric heading on left bumper press
    m_FieldRelativeButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    m_FieldRelativeButtonXbox.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
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
}
