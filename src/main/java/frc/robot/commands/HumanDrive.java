package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DriveConstants;
import frc.robot.subsystems.Telemetry;

public class HumanDrive extends Command {
    // Drive stuff
    private final SwerveDriveSubsystem drivetrain;
    private double MaxSpeed = DriveConstants.MaxSpeed;
    private double MaxAngularRate = DriveConstants.MaxAngularRate;
    private double Joystick_Deadzone = .07;
    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.RobotCentric robotDrive;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controls
    private Joystick m_Joystick1;
    private Joystick m_Joystick2;
    private final JoystickButton m_JoystickTrigger;

    // Local stuff
    private enum driveMode {
        robotRelative,
        fieldRelative1,
        fieldRelative2
    }

    private driveMode lastMode = driveMode.robotRelative;
    private driveMode currentMode;

    // This is a constructor
    public HumanDrive(Joystick m_Joystick1, Joystick m_Joystick2, SwerveDriveSubsystem drivetrain,
            SwerveRequest.FieldCentric drive, SwerveRequest.RobotCentric robotDrive) {
        this.m_Joystick1 = m_Joystick1;
        this.m_Joystick2 = m_Joystick2;
        this.drivetrain = drivetrain;
        this.drive = drive;
        this.robotDrive = robotDrive;
        m_JoystickTrigger = new JoystickButton(m_Joystick1, 1);
        currentMode = lastMode;
    }

    public double checkJoystickDeadzone(double input) {
        if (-Joystick_Deadzone < input && input < Joystick_Deadzone) {
            input = 0;
        }
        return input;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (!(currentMode == null)) {
            System.out.println(currentMode);
            System.out.println(drivetrain.getCurrentCommand());
            switch (currentMode) {
                case robotRelative:
                    drivetrain.applyRequest(() -> robotDrive
                            .withVelocityX(checkJoystickDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)) * MaxSpeed)
                            .withVelocityY(checkJoystickDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxSpeed)
                            .withRotationalRate(
                                checkJoystickDeadzone(-m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxAngularRate));
                    if (m_Joystick1.getRawButtonPressed(Constants.Thrustmaster.Left_Buttons.Top_Middle)) {
                        currentMode = driveMode.fieldRelative1;
                        lastMode = driveMode.fieldRelative1;
                    }
                    break;
                case fieldRelative1:
                    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
                    currentMode = driveMode.fieldRelative2;
                    lastMode = driveMode.fieldRelative2;
                    break;
                case fieldRelative2:
                    if (m_Joystick1.getRawButtonPressed(Constants.Thrustmaster.Left_Buttons.Top_Middle)) {
                        currentMode = driveMode.robotRelative;
                        lastMode = driveMode.robotRelative;
                    }
                    drivetrain.applyRequest(() -> drive
                            .withVelocityX(checkJoystickDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)) * MaxSpeed)
                            .withVelocityY(checkJoystickDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxSpeed)
                            .withRotationalRate(
                                checkJoystickDeadzone(-m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxAngularRate));
                    break;
            }
        }
    }
}