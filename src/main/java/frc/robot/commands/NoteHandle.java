package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class NoteHandle{
    public IntakeSubsystem intakeSubsystem;
    public ShooterSubsystem shooterSubsystem;

    public NoteHandle(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }
    public void intake() {
        shooterSubsystem.intake();
        intakeSubsystem.intake();
    }
    public void shoot() {
        shooterSubsystem.shoot();
        intakeSubsystem.shoot();
    }
    public void amp() {
        shooterSubsystem.amp();
        intakeSubsystem.amp();
    }
    public void stop() {
        shooterSubsystem.stop();
        intakeSubsystem.stop();
    }
}
