package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.DriveConstants;
// import frc.robot.subsystems.Telemetry;

public class HumanDrive extends Command {
    // Drive stuff
    private final SwerveDriveSubsystem drivetrain;
    private double MaxSpeed = DriveConstants.MaxSpeed;
    private double MaxAngularRate = DriveConstants.MaxAngularRate;
    private double Joystick_Deadzone = .05;
    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.RobotCentric robotDrive;
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final SwerveRequest.SwerveDriveBrake brake = new
    SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();
    // private final Telemetry logger = new Telemetry(MaxSpeed);
    // private boolean runOnce;

    // Controls
    private Joystick m_Joystick1;
    private Joystick m_Joystick2;
    // private final JoystickButton m_JoystickTrigger;

    // Local stuff
    boolean runOnce = true;

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
        // m_JoystickTrigger = new JoystickButton(m_Joystick1, 1);
        // m_fieldRelButton = new JoystickButton(m_Joystick1,
        // Constants.Thrustmaster.Left_Buttons.Top_Middle);

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

    }
}