package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetectionSubsystem;
import frc.robot.Constants.F310_D;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    XboxController js = new XboxController(4);
    private IntakeSubsystem m_IntakeSubsystem;

    public void execute() {
        NoteDetectionSubsystem.NoteDectectionConstants();
        if (js.getRawButton(F310_D.X)) { // X
            if (NoteDetectionSubsystem.ifAbove() == false && NoteDetectionSubsystem.ifBelow() == false) {
                IntakeSubsystem.Intake();
            } else {
                IntakeSubsystem.IntakePassover();
            }
        } else {
            IntakeSubsystem.IntakeStop();
        }
    }

}
