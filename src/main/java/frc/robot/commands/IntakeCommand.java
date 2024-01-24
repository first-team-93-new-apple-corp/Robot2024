package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetectionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.F310_D;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    XboxController opController = new XboxController(2);

    public void execute() {
        NoteDetectionSubsystem.NoteDectectionConstants();
        if (!NoteDetectionSubsystem.ifAbove() && !NoteDetectionSubsystem.ifBelow()) { 
            IntakeSubsystem.Intake();
        } else if (NoteDetectionSubsystem.ifBelow() && !NoteDetectionSubsystem.ifAbove()) {
            ShooterSubsystem.kicker();
        } else if ((NoteDetectionSubsystem.ifAbove() && !NoteDetectionSubsystem.ifBelow()) 
        || (NoteDetectionSubsystem.ifAbove() && NoteDetectionSubsystem.ifBelow())){
            IntakeSubsystem.IntakePassover();
        }else {
            IntakeSubsystem.IntakeStop();
        }
        //Manual Control
        if (opController.getRawButton(F310_D.X)){
            IntakeSubsystem.Intake();
        } else if (opController.getRawButton(F310_D.A)){
            IntakeSubsystem.IntakeConstants();
        } else if (opController.getRawButton(F310_D.B)){
            IntakeSubsystem.Outake();
        } else {
            IntakeSubsystem.IntakeStop();
        }
    }

}
