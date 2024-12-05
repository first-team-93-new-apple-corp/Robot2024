// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Auton.AutoDirector;
import frc.robot.subsystems.Auton.AutoSubsystems;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.Swerve.Telemetry;
import frc.robot.subsystems.Swerve.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 1.5; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final CommandXboxController joystick = new CommandXboxController(0);
    private SendableChooser<Command> autoChooser;
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final SwerveDriveSubsystem m_DriveSubsystem = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    AutoDirector autoDirector;

    public RobotContainer() {
        m_DriveSubsystem.configureAuto();
        autoDirector = new AutoDirector(new AutoSubsystems(m_DriveSubsystem));
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("PathPlanner AutoChooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_DriveSubsystem.setDefaultCommand(
                // m_DriveSubsystem will execute this command periodically
                m_DriveSubsystem.Commands.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) 
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
                ));

        joystick.a().whileTrue(m_DriveSubsystem.Commands.applyRequest(() -> brake));
        joystick.b().whileTrue(m_DriveSubsystem.Commands.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(m_DriveSubsystem.Commands.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(m_DriveSubsystem.Commands.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(m_DriveSubsystem.Commands.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(m_DriveSubsystem.Commands.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(m_DriveSubsystem.runOnce(() -> m_DriveSubsystem.seedFieldCentric()));

        m_DriveSubsystem.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoDirector.selection().command();
    }
}
