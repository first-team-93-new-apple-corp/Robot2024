package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {
    private PWMVictorSPX m_leftIntake;
    private PWMVictorSPX m_rightIntake;

    public IntakeSubsystem() {
        m_leftIntake = new PWMVictorSPX(0);
        m_rightIntake = new PWMVictorSPX(1);
    }
    public void intake() {
        m_leftIntake.set(0.5);
        m_rightIntake.set(0.5);
    }
    public void outtake() {
        m_leftIntake.set(-0.5); 
        m_rightIntake.set(-0.5);
    }
    public void stop() {
        m_leftIntake.set(0);
        m_rightIntake.set(0);
    }
}
