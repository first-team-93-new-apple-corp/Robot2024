package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    XboxController opController = new XboxController(2);

    @Override
    public void execute() {
        IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
        // For the Intake
        if (opController.getRawButton(Constants.F310_D.X)) { // X
            m_IntakeSubsystem.Intake();
        } else if (opController.getRawButton(Constants.F310_D.A)) { // A
            m_IntakeSubsystem.passthrough();
        } else {
            m_IntakeSubsystem.stop();
        }
    }
}