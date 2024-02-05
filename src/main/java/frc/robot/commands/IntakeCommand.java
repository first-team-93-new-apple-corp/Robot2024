package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private static XboxController opController = new XboxController(2);
    static IntakeSubsystem m_IntakeSubsystem;

    public IntakeCommand() {
        m_IntakeSubsystem = new IntakeSubsystem();
    }

    @Override
    public void execute() {
        // For the Intake
        if (opController.getRawButton(Constants.xbox.X)) { // X
            m_IntakeSubsystem.Intake();
        } else if (opController.getRawButton(Constants.xbox.A)) { // A
            m_IntakeSubsystem.passthrough();
        } else {
            m_IntakeSubsystem.stop();
        }
    }
}