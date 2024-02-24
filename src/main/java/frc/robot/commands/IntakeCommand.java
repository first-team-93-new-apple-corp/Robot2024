package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private static XboxController opController = new XboxController(2);
    IntakeSubsystem m_IntakeSubsystem;
    public int IntakeSpeed;

    public IntakeCommand(ShooterSubsystem m_shooter, IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
    }

    @Override
    public void execute() {
        // For the Intake
        if (opController.getRawButton(Constants.xbox.X)) { // X
            m_IntakeSubsystem.Intake();
        } else if (opController.getRawButton(Constants.xbox.A)) { // A
            // m_IntakeSubsystem.passthrough();
        } else {
            m_IntakeSubsystem.stop();
            m_IntakeSubsystem.resetIntakeState();
            opController.setRumble(RumbleType.kBothRumble, 0);
        }
    }
}