package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ShooterSubsystem {
    TalonFX shooterMotor;

    private double intakeSpeed = -0.2;
    private double shootSpeed = 1;
    private double ampSpeed = -0.1;
    public ShooterSubsystem() {
        shooterMotor = new TalonFX(Constants.CTRE.Shoot);
    }

    public void set(double in) {
        shooterMotor.set(in);
    }

    public void stop() {
        shooterMotor.set(0);
    }

    public void intake() {
        shooterMotor.set(intakeSpeed);
    }

    public void shoot() {
        shooterMotor.set(shootSpeed);
    }

    public void revShoot() {
        shooterMotor.set(-0.2);
    }

    public void amp() {
        shooterMotor.set(ampSpeed);
    }

}
