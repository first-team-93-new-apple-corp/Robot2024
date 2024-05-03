package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
public class ShoulderSubsystem extends SubsystemBase {
    TalonFX ShoulderL;
    TalonFX ShoulderR;
    TalonFXConfiguration m_configL;
    TalonFXConfiguration m_configR;
    PositionVoltage m_PositionVoltage = new PositionVoltage(1);
    public static final double ShoulderSpeed = 0.5;
    public void init() {
        ShoulderL = new TalonFX(1);
        ShoulderR = new TalonFX(1);
        m_configL = new TalonFXConfiguration();
        m_configR = new TalonFXConfiguration();
        m_configL.Slot0.kP = 0;
        m_configL.Slot0.kI = 0;
        m_configL.Slot0.kD = 0;
        m_configR.Slot0.kP = 0;
        m_configR.Slot0.kI = 0;
        m_configR.Slot0.kD = 0;
        ShoulderL.setInverted(true);
    }
    public void moveShoulder(double setpoint) {
        ShoulderL.setControl(m_PositionVoltage.withPosition(setpoint));
        ShoulderR.setControl(m_PositionVoltage.withPosition(setpoint));
    }
    public void stopShoulder() {
        ShoulderL.set(0);
        ShoulderR.set(0);
    }
    // Use below methods as backup
    public void shoulderUp(double ShoulderSpeed) {
        ShoulderL.set(ShoulderSpeed);
        ShoulderR.set(ShoulderSpeed);
    }
    public void shoulderDown(double ShoulderSpeed) {
        ShoulderL.set(-ShoulderSpeed);
        ShoulderR.set(-ShoulderSpeed);
    }
    // Auto 
    public void shoulderAuto() {
        ShoulderL.setControl(m_PositionVoltage);
    }
    
}