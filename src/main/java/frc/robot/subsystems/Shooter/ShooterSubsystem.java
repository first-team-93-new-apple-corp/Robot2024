package frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Shooter.IO.ShooterIO;
import frc.robot.subsystems.Shooter.IO.ShooterIO.ShooterIOInputs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

    ShooterIO m_Io;
    ShooterIOInputs m_Inputs = new ShooterIOInputs();

    public ShooterSubsystem(ShooterIO m_Io) {
        this.m_Io = m_Io;

    }

    public void shoot(double speed) {
        m_Io.shoot(speed);
    }

    public void prime() {
        m_Io.prime();
    }

    public void AutonPrime(){
        m_Io.AutonPrime();
    }

    public void shootAmp() {
        m_Io.shootAmp();
    }

    public void kicker(double KickerSpeed) {
        m_Io.kicker(KickerSpeed);
    }

    public void intakeFront() {
        m_Io.intakeFront();
    }

    public void shooterStop() {
        m_Io.shooterStop();
    }

    public void kickerStop() {
        m_Io.kickerStop();
    }
    public void ampKicker() {
        m_Io.ampKicker();
    }
    public void AmpForAuton(){
        m_Io.AmpForAuton();
    }
    public void ShootingforAuton() {
        m_Io.ShootingforAuton();
    }

    public void DribbleOutNote() {
        m_Io.DribbleOutNote();
    }

    public void increaseSpeed() {
        m_Io.increaseSpeed();
    }

    public void decreaseSpeed() {
        m_Io.decreaseSpeed();
    }

    public void ServoUp() {
        m_Io.ServoUp();
    }

    public void ServoDown() {
        m_Io.ServoDown();
    }

    @Override
    public void periodic() {
        m_Io.updateValues(m_Inputs);
        SmartDashboard.putNumber("Shooter Speed", m_Inputs.ShooterSpeed);
        SmartDashboard.putNumber("Shooter L RPM", m_Inputs.ShooterLVel);
        SmartDashboard.putNumber("Shooter R RPM", m_Inputs.ShooterRVel);
    }
}
