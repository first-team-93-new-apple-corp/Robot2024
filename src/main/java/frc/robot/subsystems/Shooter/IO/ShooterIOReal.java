package frc.robot.subsystems.Shooter.IO;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {

    private TalonFX ShooterR;
    private TalonFX ShooterL;
    static CANSparkMax KickerL;
    static CANSparkMax KickerR;
    private double SpeakerShooterSpeed;
    private final double MuzzleIntake;
    private final double AmpShooterSpeed;
    private final double KickerSpeed;
    private final double DribbleSpeed;
    private final Servo ampServo;

    public ShooterIOReal(ShooterConstants constants) {
        KickerL = new CANSparkMax(constants.KickerL, MotorType.kBrushless);
        KickerR = new CANSparkMax(constants.KickerR, MotorType.kBrushless);
        ShooterL = new TalonFX(constants.ShooterL, "rio");
        ShooterR = new TalonFX(constants.ShooterR, "rio");
        ShooterL.setInverted(true);
        KickerR.setInverted(true)
        ;
        ampServo = new Servo(8);

        SpeakerShooterSpeed = constants.SpeakerShooterSpeed;
        MuzzleIntake = constants.MuzzleIntake;
        AmpShooterSpeed = constants.AmpShooterSpeed;
        KickerSpeed = constants.KickerSpeed;
        DribbleSpeed = constants.DribbleSpeed;
    }

    @Override
    public void updateValues(ShooterIOInputs inputs) {
        inputs.ShooterSpeed = SpeakerShooterSpeed;
        inputs.ShooterLVel = ShooterL.getVelocity().getValueAsDouble();
        inputs.ShooterRVel = ShooterR.getVelocity().getValueAsDouble();
    }

    @Override
    public void shoot(double speed) {
        ShooterR.set(speed);
        ShooterL.set(speed);
    }

    @Override
    public void prime() {
        ShooterR.set(SpeakerShooterSpeed);
        ShooterL.set(SpeakerShooterSpeed);
    }

    @Override
    public void AutonPrime() {
        ShooterL.set(.60);
        ShooterR.set(.60);
    }

    @Override
    public void shootAmp() {
        ShooterR.set(AmpShooterSpeed);
        ShooterL.set(AmpShooterSpeed);
    }

    @Override
    public void kicker(double KickerSpeed) {
        KickerL.set(KickerSpeed);
        KickerR.set(KickerSpeed);
    }

    @Override
    public void intakeFront() {
        ShooterR.set(MuzzleIntake);
        ShooterL.set(MuzzleIntake);
        KickerL.set(-0.05);
        KickerR.set(-0.05);
    }

    @Override
    public void shooterStop() {
        ShooterR.set(0);
        ShooterL.set(0);
        KickerL.set(0);
        KickerR.set(0);
    }

    @Override
    public void kickerStop() {
        KickerL.set(0);
        KickerR.set(0);
    }

    @Override
    public void ampKicker() {
        KickerL.set(0.25);
        KickerR.set(0.25);
    }

    @Override
    public void AmpForAuton() {
        shootAmp();
        kicker(KickerSpeed);
    }

    @Override
    public void ShootingforAuton() {
        AutonPrime();
        kicker(KickerSpeed);
    }

    @Override
    public void DribbleOutNote() {
        shoot(DribbleSpeed);
        kicker(DribbleSpeed);
    }

    @Override
    public void increaseSpeed() {
        if (SpeakerShooterSpeed <= 0.95) {
            SpeakerShooterSpeed += 0.05; // +5%speed
        }
    }

    @Override
    public void decreaseSpeed() {
        if (SpeakerShooterSpeed >= 0.1) {
            SpeakerShooterSpeed -= 0.05; // -5%speed
        }
    }

    @Override
    public void ServoUp() {
        ampServo.set(0.1);
    }

    public void ServoDown() {
        ampServo.set(0.65);
    }
    
}
