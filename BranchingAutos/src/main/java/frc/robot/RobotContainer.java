// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withCenterOfRotation(new Translation2d(0, 1)); // Use
                                                                                                                   // open-loop
                                                                                                                   // control
                                                                                                                   // for
                                                                                                                   // drive
                                                                                                                   // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // Controls
    private final CommandXboxController Xbox = new CommandXboxController(2);
    private final CommandJoystick leftStick = new CommandJoystick(0);
    private final CommandJoystick RightStick = new CommandJoystick(1);

    // Auton
    private SendableChooser<Command> autoChooser;
    AutoDirector autoDirector;

    // Logging
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // LEDs
    LED m_LED = new LED();
    private LEDCommand LEDCommand = m_LED.new LEDCommand();

    public RobotContainer() {
        configureBindings();
    }

    private double POVDistance = .45;
    private double POVDistanceDiagonal = Math.sqrt(2 * (Math.pow(POVDistance, 2)));
    private Translation2d[] POVs = {
            new Translation2d(0, 0), // Default
            new Translation2d(0, POVDistance), // left 1
            new Translation2d(POVDistance, 0), // Up 2
            new Translation2d(0, -POVDistance), // Right 3
            new Translation2d(-POVDistance, 0), // Down 4
            new Translation2d(POVDistanceDiagonal, POVDistanceDiagonal), // up left 5
            new Translation2d(POVDistanceDiagonal, -POVDistanceDiagonal), // up right 6
            new Translation2d(-POVDistanceDiagonal, POVDistanceDiagonal), // down left 7
            new Translation2d(-POVDistanceDiagonal, -POVDistanceDiagonal) // down right 8
    };

    public Translation2d getPOV() {
        if (leftStick.povLeft().getAsBoolean()) {
            return POVs[1];
        } else if (leftStick.povUp().getAsBoolean()) {
            return POVs[2];
        } else if (leftStick.povRight().getAsBoolean()) {
            return POVs[3];
        } else if (leftStick.povDown().getAsBoolean()) {
            return POVs[4];
        } else if (leftStick.povUpLeft().getAsBoolean()) {
            return POVs[5];
        } else if (leftStick.povUpRight().getAsBoolean()) {
            return POVs[6];
        } else if (leftStick.povDownLeft().getAsBoolean()) {
            return POVs[7];
        } else if (leftStick.povDownRight().getAsBoolean()) {
            return POVs[8];
        } else {
            return POVs[0];
        }
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
        // m_DriveSubsystem.setDefaultCommand(
        // m_DriveSubsystem.Commands.applyRequest(() ->
        // drive.withVelocityX(-Xbox.getLeftY() * MaxSpeed)
        // .withVelocityY(-Xbox.getLeftX() * MaxSpeed)
        // .withRotationalRate(-Xbox.getRightX() *
        // MaxAngularRate).withCenterOfRotation(getPOV())));
        m_DriveSubsystem.setDefaultCommand(
                m_DriveSubsystem.Commands.applyRequest(() -> drive.withVelocityX(-leftStick.getY() * MaxSpeed)
                        .withVelocityY(-leftStick.getX() * MaxSpeed)
                        .withRotationalRate(-RightStick.getX() * MaxAngularRate)
                        .withCenterOfRotation(getPOV())
                        ));
        leftStick.button(11).onTrue(m_DriveSubsystem.runOnce(() -> m_DriveSubsystem.seedFieldCentric()));
        // Xbox.a().whileTrue(m_DriveSubsystem.Commands.applyRequest(() -> brake));
        // Xbox.b().whileTrue(m_DriveSubsystem.Commands.applyRequest(
        //     () -> point.withModuleDirection(new Rotation2d(-Xbox.getLeftY(), -Xbox.getLeftX()))));
        // Xbox.leftBumper().onTrue(m_DriveSubsystem.runOnce(() -> m_DriveSubsystem.seedFieldCentric()));
        Xbox.x().onTrue(LEDCommand.test(10, Color.kGreen, Color.kBlack, 25, 75).andThen(LEDCommand.off()));
        Xbox.b().onTrue(LEDCommand.shoot().andThen(LEDCommand.off()));
        Xbox.y().onTrue(LEDCommand.test2().andThen(LEDCommand.off()));
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
