package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    TalonFX ShoulderL;
    TalonFX ShoulderR;
    DutyCycleEncoder m_Encoder;
    TalonFXConfiguration m_configL;
    TalonFXConfiguration m_configR;
    PositionVoltage m_PositionVoltage = new PositionVoltage(1);
    NeutralOut m_NeutralOut = new NeutralOut();
    double position;
    double setpoint;
    double kP = 0.5;
    double kI = 0;
    double kD = 0;
    double kG = 0.23;
    double kV = 5.39;
    double kA = 0.01;
    public static final double ShoulderSpeed = 0.5;

    public void init() {
        m_Encoder = new DutyCycleEncoder(Constants.Sensors.DIO.ThroughBoreEncoder);
        ShoulderL = new TalonFX(Constants.CTRE.L_Shoulder);
        ShoulderR = new TalonFX(Constants.CTRE.R_Shoulder);
        m_configL = new TalonFXConfiguration();
        m_configR = new TalonFXConfiguration();
        m_configL.CurrentLimits.StatorCurrentLimit = 20;
        m_configR.CurrentLimits.StatorCurrentLimit = 20;
        m_configL.CurrentLimits.StatorCurrentLimitEnable = true;
        m_configR.CurrentLimits.StatorCurrentLimitEnable = true;
        m_configL.Slot0.kP = kP;
        m_configL.Slot0.kI = kI;
        m_configL.Slot0.kD = kD;
        m_configL.Slot0.kG = kG;
        m_configL.Slot0.kV = kV;
        m_configL.Slot0.kA = kA;
        m_configL.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_configR.Slot0.kP = kP;
        m_configR.Slot0.kI = kI;
        m_configR.Slot0.kD = kD;
        m_configR.Slot0.kG = kG;
        m_configR.Slot0.kV = kV;
        m_configR.Slot0.kA = kA;
        m_configR.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        ShoulderL.getConfigurator().apply(m_configL);
        ShoulderR.getConfigurator().apply(m_configR);
        ShoulderL.setInverted(true);
        ShoulderR.setInverted(true);
        ShoulderL.setNeutralMode(NeutralModeValue.Brake);
        ShoulderR.setNeutralMode(NeutralModeValue.Brake);

    }

    public void toSetpoint(double setpoint) {
        this.setpoint = setpoint;
        setpoint = setpoint * 2048 * 300;
        ShoulderL.setControl(m_PositionVoltage.withPosition(setpoint));
        ShoulderR.setControl(m_PositionVoltage.withPosition(setpoint));
    }

    public void lock() {
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

    public double getPosition() {
        position = m_Encoder.get();
        // 5.67 - (int of 5.67) 5 = 0.67
        position -= (int) position;
        return position;
    }

    public boolean atSetpoint() {
        SmartDashboard.putNumber("Setpoint", setpoint);
        return setpoint - 0.05 < position && position < setpoint + 0.05;
    }
}