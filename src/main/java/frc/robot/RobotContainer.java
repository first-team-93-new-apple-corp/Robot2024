// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.TunerConstants;

public class RobotContainer extends TimedRobot {
  private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SendableChooser<Command> autoChooser;
  public final double MaxSpeed = DriveConstants.MaxSpeed;
  public final double MaxAngularRate = DriveConstants.MaxAngularRate;
  private final Joystick m_Joystick1 = new Joystick(0);
  private final Joystick m_Joystick2 = new Joystick(1);
  private final boolean Limit = true;
  private final JoystickButton m_JoystickTrigger = new JoystickButton(m_Joystick1, 1);
  private final JoystickButton m_fieldRelButton = new JoystickButton(m_Joystick1,
      Constants.Thrustmaster.Left_Buttons.Top_Middle);
  private final JoystickButton m_JoystickButton2 = new JoystickButton(m_Joystick1, 2);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Configures the bindings to drive / control the swerve drive :)
  public void configureBindings() {

    // public void RotationPoints(Joystick m_Joystick2) {
    // POVButton pov0 = new POVButton(m_Joystick2, 0); // front
    // POVButton pov45 = new POVButton(m_Joystick2, 45); // fr wheel
    // POVButton pov90 = new POVButton(m_Joystick2, 90); // right
    // POVButton pov135 = new POVButton(m_Joystick2, 135); // br wheel
    // POVButton pov180 = new POVButton(m_Joystick2, 180); // back
    // POVButton pov225 = new POVButton(m_Joystick2, 225);// bl wheel
    // POVButton pov270 = new POVButton(m_Joystick2, 270);// left
    // POVButton pov315 = new POVButton(m_Joystick2, 315); // fl wheel
    // POVButton povCenter = new POVButton(m_Joystick2, -1);

    // // SmartDashboard.putNumber("Limit", Limit);
    // if (!(m_Joystick2.getRawAxis(0) <= 0.1 && m_Joystick2.getRawAxis(0) >= -0.1)) {
    //   if (Limit) {
    //     Limit = false;
    //     // SmartDashboard.putNumber("Limit", Limit);
    //     if (pov0.getAsBoolean()) {
    //       DriveConstants.dCenter = DriveConstants.Front
    //           .rotateBy(Rotation2d.fromDegrees(-1 * SwerveDriveSubsystem.getHeading()));
    //     } else if (pov45.getAsBoolean()) {
    //       DriveConstants.dCenter = DriveConstants.Location_FR
    //           .rotateBy(Rotation2d.fromDegrees(-1 * SwerveDriveSubsystem.getHeading()));
    //     } else if (pov90.getAsBoolean()) {
    //       DriveConstants.dCenter = DriveConstants.Right
    //           .rotateBy(Rotation2d.fromDegrees(-1 * SwerveDriveSubsystem.getHeading()));
    //     } else if (pov135.getAsBoolean()) {
    //       DriveConstants.dCenter = DriveConstants.Location_BR
    //           .rotateBy(Rotation2d.fromDegrees(-1 * SwerveDriveSubsystem.getHeading()));
    //     } else if (pov180.getAsBoolean()) {
    //       DriveConstants.dCenter = DriveConstants.Back
    //           .rotateBy(Rotation2d.fromDegrees(-1 * SwerveDriveSubsystem.getHeading()));
    //     } else if (pov225.getAsBoolean()) {
    //       DriveConstants.dCenter = DriveConstants.Location_BL
    //           .rotateBy(Rotation2d.fromDegrees(-1 * SwerveDriveSubsystem.getHeading()));
    //     } else if (pov270.getAsBoolean()) {
    //       DriveConstants.dCenter = DriveConstants.Left
    //           .rotateBy(Rotation2d.fromDegrees(-1 * SwerveDriveSubsystem.getHeading()));
    //     } else if (pov315.getAsBoolean()) {
    //       DriveConstants.dCenter = DriveConstants.Location_FL
    //           .rotateBy(Rotation2d.fromDegrees(-1 * SwerveDriveSubsystem.getHeading()));
    //     } else {
    //       DriveConstants.dCenter = new Translation2d(0, 0);
    //     }
    //   } else {
    //     Limit = false;
    //     if (povCenter.getAsBoolean()) {
    //       DriveConstants.dCenter = new Translation2d(0, 0);
    //     }
    //   }
    // } else {
    //   Limit = true;
    // }

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
    drivetrain.configAuto();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }
  
  public Command getAutonomousCommand() {
    drivetrain.configAuto();
    return autoChooser.getSelected();
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    // // Create a path following command using AutoBuilder. This will also trigger event markers.
    // return AutoBuilder.followPath(path);
  }
}
