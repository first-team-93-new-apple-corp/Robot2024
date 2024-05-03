package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    TalonFX m_motor;
    TalonFXConfiguration m_config;
    NeutralOut m_NeutralOut = new NeutralOut();
    PositionVoltage m_PositionVoltage = new PositionVoltage(0);
    double position;
    double restoredPos;

    /*
     * Restoring position:
     * Will use custom param 0 (-32768 through 32767)
     * if it takes, say, idk 25.4 rotations to fully extend the elevator
     * one rotation is 2048 ticks
     * 2048 * 3 * 25.4 = 156057.6 / 5 = 31211.52, which is well within the range
     * without compromising too much
     * 
     */
    public void init() {
        // Motor
        m_motor = new TalonFX(Constants.CTRE.RIO.Elevator);

        // Config
        m_config = new TalonFXConfiguration();
        // TODO: Tune for actual elevator
        m_config.Slot0.kP = 0.05;
        m_config.Slot0.kI = 0;
        m_config.Slot0.kD = 0;
        m_config.CurrentLimits.StatorCurrentLimit = 50;
        m_config.Audio.AllowMusicDurDisable = true;
        m_config.Audio.BeepOnBoot = false;
        m_config.Audio.BeepOnBoot = false;
        restoredPos = m_config.CustomParams.CustomParam0;
        // Control Requests
        m_PositionVoltage.Slot = 0;
        m_PositionVoltage.EnableFOC = false;

        // Applying Config
        m_motor.getConfigurator().apply(m_config);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motor.setPosition(restoredPos);
    }

    public void toSetpoint(double setpoint) {
        m_motor.setControl(m_PositionVoltage.withPosition(setpoint));
    }

    public void stop() {
        m_motor.setControl(m_NeutralOut);
    }

    public void savePosition() {

    }

    public double toStorage() {
        return m_motor.getPosition().getValueAsDouble() / 5;
    }

    public double fromStorage(double input) {
        return input * 5;
    }

    @Override
    public void periodic() {

    }
}