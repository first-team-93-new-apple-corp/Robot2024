package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    // CANcoder m_fakeCANcoder = new CANcoder(93);
    // CANcoderSimState m_CaNcoderSimState = new CANcoderSimState(m_fakeCANcoder);
    DutyCycleEncoder m_Encoder;
    TalonFXConfiguration m_configL;
    TalonFXConfiguration m_configR;
    // PositionVoltage m_PositionVoltage = new PositionVoltage(1);
    MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0);
    MotionMagicConfigs m_MotionMagicConfigs = new MotionMagicConfigs();
    NeutralOut m_NeutralOut = new NeutralOut();
    double motorPos;
    double position;
    double setpoint;

    // double kP = 1.00;
    // double kI = 0;
    // double kD = 0.4;
    // // double kG = 0.23; // 0.23
    // double kV = 5.39;
    // double kA = 0.01;
    // double kS = 0.1;
    public static final double ShoulderSpeed = 0.5;

    int periodicTimer = 0;

    public void init() {

        // Definitions
        m_Encoder = new DutyCycleEncoder(Constants.Sensors.DIO.ThroughBoreEncoder);

        ShoulderL = new TalonFX(Constants.CTRE.L_Shoulder);
        ShoulderR = new TalonFX(Constants.CTRE.R_Shoulder);

        // Config
        m_configL = new TalonFXConfiguration();
        m_configR = new TalonFXConfiguration();

        m_configL.CurrentLimits.StatorCurrentLimit = 20;
        m_configR.CurrentLimits.StatorCurrentLimit = 20;

        m_configL.CurrentLimits.StatorCurrentLimitEnable = true;
        m_configR.CurrentLimits.StatorCurrentLimitEnable = true;

        m_configL.Slot0.kP = 0;
        m_configL.Slot0.kI = 0;
        m_configL.Slot0.kD = 0;
        m_configL.Slot0.kV = 0.05; // 5.39
        m_configL.Slot0.kA = 0;
        m_configL.Slot0.kS = 0.175;

        // m_configL.Slot0.kG = 0.1;

        m_MotionMagicConfigs = m_configL.MotionMagic;
        m_MotionMagicConfigs.MotionMagicCruiseVelocity = 40; // Target cruise velocity of 80 rps
        m_MotionMagicConfigs.MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds)
        // m_MotionMagicConfigs.MotionMagicJerk = 5;

        // Motor Setup
        ShoulderL.getConfigurator().apply(m_configL);
        ShoulderR.getConfigurator().apply(m_configR);

        ShoulderL.setInverted(false);
        ShoulderR.setInverted(false);

        ShoulderL.setNeutralMode(NeutralModeValue.Brake);
        ShoulderR.setNeutralMode(NeutralModeValue.Brake);

        // Set right motor to follow left to simplify
        ShoulderR.setControl(new Follower(ShoulderL.getDeviceID(), false));
        m_Encoder.setPositionOffset(0.5);

        ShoulderL.setPosition(m_Encoder.get() * 300);
        ShoulderR.setPosition(m_Encoder.get() * 300);

    }

    public void toSetpoint(double setpointin) {
        setpoint = setpointin;
        ShoulderL.setControl(m_MotionMagicVoltage.withPosition(setpoint));
    }

    public void lock() {
        ShoulderL.set(0);
    }

    // Auto
    public void shoulderAuto() {
        ShoulderL.setControl(m_MotionMagicVoltage);
    }

    public double getPosition() {
        position = m_Encoder.get();
        return position;
    }

    public boolean atSetpoint() {
        SmartDashboard.putNumber("Setpoint", setpoint);

        return ShoulderL.getPosition().getValueAsDouble() < setpoint + 0.5 && ShoulderL.getPosition().getValueAsDouble() > setpoint - 0.5;

    }

    public double getLeft() {
        return ShoulderL.getPosition().getValueAsDouble();
    }

    public double getRight() {
        return ShoulderR.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        periodicTimer++;
        if (periodicTimer > 50) {
            ShoulderL.setPosition(m_Encoder.get() * 300);
            ShoulderR.setPosition(m_Encoder.get() * 300);
            periodicTimer = 0;
        }
        if (atSetpoint()) {
            lock();
        }
    }

}