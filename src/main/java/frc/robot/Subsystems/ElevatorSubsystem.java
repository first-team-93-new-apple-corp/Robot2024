package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    TalonFX m_motor;
    TalonFXConfiguration m_config;
    NeutralOut m_NeutralOut;
    PositionVoltage m_PositionVoltage;
    DutyCycleEncoder m_Encoder;

    public void init() {
        //Init variables
        m_Encoder = new DutyCycleEncoder(0);
        m_PositionVoltage = new PositionVoltage(0);
        m_NeutralOut = new NeutralOut();
        
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
        
        // Control Requests
        m_PositionVoltage.Slot = 0;
        m_PositionVoltage.EnableFOC = false;

        // Applying Config
        m_motor.getConfigurator().apply(m_config);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        
    }

    public void toSetpoint(double setpoint) {
        m_motor.setControl(m_PositionVoltage.withPosition(setpoint));
    }

    public void stop() {
        m_motor.setControl(m_NeutralOut);
    }


    @Override
    public void periodic() {

    }
}