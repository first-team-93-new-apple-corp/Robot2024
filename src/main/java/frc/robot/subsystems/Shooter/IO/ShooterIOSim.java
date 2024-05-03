package frc.robot.subsystems.Shooter.IO;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.MotorSim;

public class ShooterIOSim implements ShooterIO {

    private MotorSim ShooterR;
    private MotorSim ShooterL;
    static MotorSim KickerL;
    static MotorSim KickerR;
    private double SpeakerShooterSpeed;
    private final double MuzzleIntake;
    private final double AmpShooterSpeed;
    private final double KickerSpeed;
    private final double DribbleSpeed;

    public ShooterIOSim(ShooterConstants constants) {
        KickerL = new MotorSim(75);
        KickerR = new MotorSim(75);
        ShooterL = new MotorSim(100);
        ShooterR = new MotorSim(100);
        ShooterL.setInverted(true);
        KickerR.setInverted(true);

        SpeakerShooterSpeed = constants.SpeakerShooterSpeed;
        MuzzleIntake = constants.MuzzleIntake;
        AmpShooterSpeed = constants.AmpShooterSpeed;
        KickerSpeed = constants.KickerSpeed;
        DribbleSpeed = constants.DribbleSpeed;
    }

    @Override
    public void updateValues(ShooterIOInputs inputs) {
        ShooterL.periodic();
        ShooterR.periodic();
        KickerL.periodic();
        KickerR.periodic();
        inputs.ShooterSpeed = SpeakerShooterSpeed;
        inputs.ShooterLVel = ShooterL.getVelocity();
        inputs.ShooterRVel = ShooterR.getVelocity();
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
        // TODO Auto-generated method stub
    }

    @Override
    public void ServoDown() {
        // TODO Auto-generated method stub
    }
    
}
