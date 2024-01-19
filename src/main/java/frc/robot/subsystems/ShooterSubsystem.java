package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    static TalonFX ShooterR = new TalonFX(05);
    static TalonFX ShooterL = new TalonFX(11);
    static CANSparkMax IntoShooter = new CANSparkMax(3, MotorType.kBrushless);
    static double SpeakerShooterSpeed = 0.45;
    static double currentspeed;
    static double MuzzleIntake = 0.25;
    final static double AmpShooterSpeed = 0.3;
    final static double KickerSpeed = -1;

    public static void shootSpeaker() {
        System.out.println("shooting speaker");
        ShooterR.set(-SpeakerShooterSpeed);
        ShooterL.set(SpeakerShooterSpeed);

    }

    public static void shootAmp() {
        ShooterR.set(-AmpShooterSpeed);
        ShooterL.set(AmpShooterSpeed);
    }

    public static void shootIntake() {
        IntoShooter.set(KickerSpeed);
    }

    public static void shootMuzzle() {
        ShooterR.set(MuzzleIntake);
        ShooterL.set(-MuzzleIntake);
    }

    public static void shootStop() {
        ShooterR.set(0);
        ShooterL.set(0);
    }

    public static void shootIntakeStop() {
        IntoShooter.set(0);
    }

    public static void shootPlus() {
        SpeakerShooterSpeed += 0.05; // +5%speed
        SmartDashboard.putNumber("CurrentSpeed", SpeakerShooterSpeed);

    }

    public static void shootMinus() {
        SpeakerShooterSpeed -= 0.05; // -5%speed
        SmartDashboard.putNumber("CurrentSpeed", SpeakerShooterSpeed);

    }

    public static void shootConstants() {
        SmartDashboard.putNumber("Shooter Speed", SpeakerShooterSpeed);
        SmartDashboard.putNumber("ShooterRmp", ShooterR.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 1 Temp", ShooterL.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter 2 Temp", ShooterR.getDeviceTemp().getValueAsDouble());
    }

    @Override
    public void periodic() {
       
    }
}
