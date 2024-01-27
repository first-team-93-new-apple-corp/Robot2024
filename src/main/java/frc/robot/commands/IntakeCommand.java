package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.F310_D;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    XboxController opController = new XboxController(2);
    private IntakeCommand m_IntakeCommand;

    public void execute() {
        if (opController.getRawButton(F310_D.X)) { // X
            IntakeSubsystem.Intake();
        } else if (opController.getRawButton(F310_D.A)) { // A
            IntakeSubsystem.IntakePassover();
        } else if (opController.getRawButton(F310_D.B)) {
            IntakeSubsystem.Outake();
        } else {
            IntakeSubsystem.IntakeStop();
        }
    }
}
