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
    }
    public void prime() {
        intakeSubsystem.shoot();
    }
    public void revShoot(){
        shooterSubsystem.revShoot();
        intakeSubsystem.revShoot();
    }
    public void amp() {
        shooterSubsystem.amp();
        // intakeSubsystem.amp();
    }
    public void stop() {
        shooterSubsystem.stop();
        intakeSubsystem.stop();
    }
    public boolean hasNote() {
        return intakeSubsystem.hasNote();
    }
}
