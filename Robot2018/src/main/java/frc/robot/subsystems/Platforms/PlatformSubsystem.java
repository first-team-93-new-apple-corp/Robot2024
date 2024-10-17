package frc.robot.subsystems.Platforms;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class PlatformSubsystem implements Subsystem {
    private VictorSPX m_leftMotor;
    private VictorSPX m_rightMotor;

    double windSpeed = 0.15;
    public PlatformSubsystem() {
        m_leftMotor = new VictorSPX(2);
        m_rightMotor = new VictorSPX(3);

        m_leftMotor.setNeutralMode(NeutralMode.Brake);
        m_rightMotor.setNeutralMode(NeutralMode.Brake);
        //Positive wind (I think)
    }
    public void windUpLeft() {
        m_leftMotor.set(VictorSPXControlMode.PercentOutput, windSpeed);
    }
    public void windUpRight() {
        m_rightMotor.set(VictorSPXControlMode.PercentOutput, windSpeed);
    }
    public void windDownLeft() {
        m_leftMotor.set(VictorSPXControlMode.PercentOutput, -windSpeed);
    }
    public void windDownRight() {
        m_rightMotor.set(VictorSPXControlMode.PercentOutput, -windSpeed);
    }

}
