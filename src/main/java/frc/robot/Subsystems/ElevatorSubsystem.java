package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    TalonFX m_motor;
    TalonFXConfiguration m_config;
    NeutralOut m_NeutralOut;
    PositionVoltage m_PositionVoltage;
    AnalogInput m_HallEffect;
    double setpoint;

    public void init() {
        SmartDashboard.putNumber("Elevator Setpoint", 35);
        // Init variables
        m_HallEffect = new AnalogInput(Constants.Sensors.AnalogIn.HallEffect);
        m_PositionVoltage = new PositionVoltage(0);
        m_NeutralOut = new NeutralOut();

        // Motor
        m_motor = new TalonFX(Constants.CTRE.Elevator);

        // Config
        m_config = new TalonFXConfiguration();
        m_config.Slot0.kP = 1.75;
        m_config.Slot0.kI = 0;
        m_config.Slot0.kD = 0.1;
        m_config.Slot0.kG = 0.8; // 1.3
        m_config.Slot0.kV = 2.83;
        m_config.Slot0.kA = 0.16;
        m_config.CurrentLimits.StatorCurrentLimit = 25;
        m_config.Audio.AllowMusicDurDisable = true;
        m_config.Audio.BeepOnBoot = false;
        m_config.Audio.BeepOnBoot = false;

        // Control Requests
        m_PositionVoltage.Slot = 0;
        m_PositionVoltage.EnableFOC = false;

        // Applying Config
        m_motor.getConfigurator().apply(m_config);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        
    }

    public Command toSetpoint(double setpoint) {
        this.setpoint = setpoint;
        return this.runOnce(() -> m_motor.setControl(m_PositionVoltage.withPosition(setpoint)));
    }

    public Command lock() {
        return this.runOnce(() -> m_motor.setControl(m_NeutralOut));
    }

    public double getPostition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return setpoint - 0.5 < m_motor.getPosition().getValueAsDouble()
                && setpoint + 0.5 > m_motor.getPosition().getValueAsDouble();
    }

    public boolean getZero() {
        return m_HallEffect.getValue() < 50;
    }
    private void zeroing() {
        if(!getZero()) {
            m_motor.set(-0.1);
        } else {
            m_motor.set(0);
        }
    }
    public Command zeroTelescope() {
    // public void zeroTelescope() {
        return this.run(() -> zeroing());
        // zeroing();

    }
    public void checkZero() {
        if (getZero()) {
            m_motor.setPosition(0);
        }
    }
}