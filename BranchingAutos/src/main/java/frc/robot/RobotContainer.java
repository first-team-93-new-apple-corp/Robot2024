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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Auton.AutoDirector;
import frc.robot.subsystems.Auton.AutoSubsystems;
import frc.robot.subsystems.LED.LEDCommand;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.Swerve.Telemetry;
import frc.robot.subsystems.Swerve.TunerConstants;

public class RobotContainer {
    // Drivetrain
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 1.5; // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    public final SwerveDriveSubsystem m_DriveSubsystem = TunerConstants.createDrivetrain();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // Controls
    private final CommandXboxController joystick = new CommandXboxController(0);

    // Auton
    private SendableChooser<Command> autoChooser;
    AutoDirector autoDirector;

    // Logging
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // LEDs
    private LED m_LED = new LED();
    private LEDCommand LEDCommand = m_LED.new LEDCommand();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // AUTON
        m_DriveSubsystem.configureAuto();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("PathPlanner AutoChooser", autoChooser);
        autoDirector = new AutoDirector(new AutoSubsystems(m_DriveSubsystem));

        // DRIVE
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_DriveSubsystem.setDefaultCommand(
                m_DriveSubsystem.Commands.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
        joystick.a().whileTrue(m_DriveSubsystem.Commands.applyRequest(() -> brake));
        joystick.b().whileTrue(m_DriveSubsystem.Commands.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
        joystick.leftBumper().onTrue(m_DriveSubsystem.runOnce(() -> m_DriveSubsystem.seedFieldCentric()));
        m_DriveSubsystem.registerTelemetry(logger::telemeterize);

        // SYSID ROUTINES
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(m_DriveSubsystem.Commands.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(m_DriveSubsystem.Commands.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(m_DriveSubsystem.Commands.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(m_DriveSubsystem.Commands.sysIdQuasistatic(Direction.kReverse));

    }

    public Command getAutonomousCommand() {
        return autoDirector.selection().command();
    }
    public Command getIdleLEDs() {
        return LEDCommand.applyColorCycle(4, Color.kBlue, Color.kRed);
    }
}
