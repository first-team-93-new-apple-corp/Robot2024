package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class IntakeSubsystem {
    public TalonFX intakeMotor;

    private double intakeSpeed = 0.5;
    private double shootSpeed = 1;
    private double ampSpeed = -0.2;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.CTRE.Intake);
        intakeMotor.setInverted(true);
    }
    public void set(double in) {
        intakeMotor.set(in);
    }
    public void stop() {
        intakeMotor.set(0);
    }
    public void intake() {
        intakeMotor.set(intakeSpeed);
    }
    public void shoot() {
        intakeMotor.set(shootSpeed);
    }
    public void revShoot(){
        intakeMotor.set(-0.2);
    }
    public void amp() {
        intakeMotor.set(ampSpeed);
    }
}
