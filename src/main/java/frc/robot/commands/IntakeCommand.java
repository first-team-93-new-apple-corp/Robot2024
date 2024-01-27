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
        IntakeSubsystem.IntakeConstants();
        
        // Manual Control

        if (opController.getRawButton(F310_D.X)) {
            IntakeSubsystem.Intake();
            System.out.println("Intaking");
        } else if (opController.getRawButton(F310_D.A)) {
            IntakeSubsystem.IntakePassover();
        } else if (opController.getRawButton(F310_D.B)) {
            IntakeSubsystem.Outake();
        } else if (opController.getRawButton(F310_D.Y)) {
            IntakeSubsystem.IntakeStop();
        } else {

            if (!NoteDetectionSubsystem.ifAbove() && !NoteDetectionSubsystem.ifBelow()) {
                System.out.println("Sensor Intaking");
                IntakeSubsystem.Intake();
            } else {
                IntakeSubsystem.IntakePassover();
            }

            // Shooter Section

            if (NoteDetectionSubsystem.ifBelow() && !NoteDetectionSubsystem.ifAbove()) {
                ShooterSubsystem.kicker();
            }
        }
    }
}
