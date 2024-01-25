// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HumanDrive;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  public final double MaxSpeed = DriveConstants.MaxSpeed;
  public final double MaxAngularRate = DriveConstants.MaxAngularRate;
  private final Joystick m_Joystick1 = new Joystick(0);
  private final Joystick m_Joystick2 = new Joystick(1);
  private final JoystickButton m_JoystickTrigger = new JoystickButton(m_Joystick1, 1);
  private final JoystickButton m_fieldRelButton = new JoystickButton(m_Joystick1,
      Constants.Thrustmaster.Left_Buttons.Top_Middle);
  private final JoystickButton m_JoystickButton2 = new JoystickButton(m_Joystick1, 2);
  private final SwerveDriveSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private ChoreoTrajectory traj;
  private Field2d m_field = new Field2d();

  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private HumanDrive m_HumanDrive = new HumanDrive(m_Joystick1, m_Joystick2, drivetrain, drive, robotDrive);
  private VisionSubsystem m_VisionSubsystem = new VisionSubsystem(drivetrain);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Configures the bindings to drive / control the swerve drive :)
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(
                m_HumanDrive.checkJoystickDeadzone(
                    -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y))
                    * MaxSpeed)
            .withVelocityY(
                m_HumanDrive.checkJoystickDeadzone(
                    -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x))
                    * MaxSpeed)
            .withRotationalRate(
                m_HumanDrive.checkJoystickDeadzone(
                    -m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x))
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
    m_VisionSubsystem.updateValues();
    traj = Choreo.getTrajectory("TEST.1");

    m_field.getObject("traj").setPoses(
        traj.getInitialPose(), traj.getFinalPose());
    m_field.getObject("trajPoses").setPoses(
        traj.getPoses());

    SmartDashboard.putData(m_field);

  }

  public boolean mirrorAuto() {
    if (DriverStation.Alliance.Blue == DriverStation.getAlliance().get()) {
      return false;
    } else {
      return true;
    }
  }

  public Command getAutonomousCommand() {
    var thetaController = new PIDController(0.005, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // drivetrain.resetOdometry(traj.getInitialPose());

    Command swerveCommand = Choreo.choreoSwerveCommand(
        traj, // Choreo trajectory from above
        () -> drivetrain.getPose2D(), // A function that returns the current field-relative pose of the robot: your
        // wheel or vision odometry
        new PIDController(0.005, 0.0, 0.0), // PIDController for field-relative X
                                            // translation (input: X error in meters,
                                            // output: m/s).
        new PIDController(0.005, 0.0, 0.0), // PIDController for field-relative Y
                                            // translation (input: Y error in meters,
                                            // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        // (ChassisSpeeds speeds) -> drivetrain.drive( // needs to be robot-relative
        // speeds.vxMetersPerSecond,
        // speeds.vyMetersPerSecond,
        // speeds.omegaRadiansPerSecond),
        (ChassisSpeeds speeds) -> drivetrain.setControl(drive
            .withVelocityX(
                speeds.vxMetersPerSecond)
            .withVelocityY(
                speeds.vyMetersPerSecond)
            .withRotationalRate(
                speeds.omegaRadiansPerSecond)),
        () -> mirrorAuto(), // Whether or not to mirror the path based on alliance (this assumes the path is
                            // created for the blue alliance)
        drivetrain // The subsystem(s) to require, typically your drive subsystem only
    );
    return Commands.sequence(
        drivetrain.runOnce(() -> drivetrain.seedFieldRelative()),
        drivetrain.runOnce(() -> drivetrain.resetOdometry(traj.getInitialPose())),
        swerveCommand,
        drivetrain.runOnce(
            () -> drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0))));
  }

  public Command getTeleopCommand() {
    return m_HumanDrive;
  }
}
