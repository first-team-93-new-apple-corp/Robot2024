package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX ShooterR = new TalonFX(Constants.CTRE.RIO.R_Shoot, "rio");
    TalonFX ShooterL = new TalonFX(Constants.CTRE.RIO.L_Shoot, "rio");
    static CANSparkMax KickerL;
    static CANSparkMax KickerR;
    double SpeakerShooterSpeed = 0.45;
    double currentspeed;
    final double MuzzleIntake = -0.10;
    final double AmpShooterSpeed = 0.3;
    final double KickerSpeed = 1;

    public ShooterSubsystem() {
        KickerL = new CANSparkMax(Constants.REV.L_Kicker, MotorType.kBrushless);
        KickerR = new CANSparkMax(Constants.REV.R_Kicker, MotorType.kBrushless);
        ShooterL.setInverted(true);
        ShooterR.setInverted(false);
        KickerR.setInverted(true);
    }

    public void prime() {
        ShooterR.set(SpeakerShooterSpeed);
        ShooterL.set(SpeakerShooterSpeed);
    }

    public void shootAmp() {
        ShooterR.set(AmpShooterSpeed);
        ShooterL.set(AmpShooterSpeed);
    }

    public void kicker() {
        KickerL.set(KickerSpeed);
        KickerR.set(KickerSpeed);
    }

    public void intakeFront() {
        ShooterR.set(MuzzleIntake);
        ShooterL.set(MuzzleIntake);
        KickerL.set(-0.05);
        KickerR.set(-0.05);
    }

    public void shooterStop() {
        ShooterR.set(0);
        ShooterL.set(0);
    }

    public void kickerStop() {
        KickerL.set(0);
        KickerR.set(0);
    }

    public void increaseSpeed() {
        if (SpeakerShooterSpeed <= 0.95) {
            SpeakerShooterSpeed += 0.05; // +5%speed
        }
        SmartDashboard.putNumber("CurrentSpeed", SpeakerShooterSpeed);

    }

    public void decreaseSpeed() {
        if (SpeakerShooterSpeed >= 0.1) {
            SpeakerShooterSpeed -= 0.05; // -5%speed
        }
        SmartDashboard.putNumber("CurrentSpeed", SpeakerShooterSpeed);

    }

    public Command AutoShoot() {
        System.out.println("Shooting Note");
        return this.run(() -> prime());
    }

    public Command AutoStopShoot() {
        System.out.println("Stopping Shooter");
        return this.run(() -> shooterStop());
    }

    public Command AutoKicker() {
        System.out.println("Kicking Note");
        return this.run(() -> kicker());
    }

    public Command AutoStopKicker() {
        System.out.println("Stopping Kicker");
        return this.run(() -> kickerStop());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", SpeakerShooterSpeed);
        SmartDashboard.putNumber("Shooter L RPM", ShooterR.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter R RPM", ShooterR.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter L Temp", ShooterL.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter R Temp", ShooterR.getDeviceTemp().getValueAsDouble());
    }
}
