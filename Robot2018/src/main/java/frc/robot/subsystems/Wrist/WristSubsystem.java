package frc.robot.subsystems.Wrist;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class WristSubsystem implements Subsystem {
    //TODO: Quadrature Encoder? 
    // https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html
    private TalonSRX wristMotor;
    
    public WristSubsystem() {
        wristMotor = new TalonSRX(12);
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }
    public void moveUp() {
        wristMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
    }
    public void moveDown() {
        wristMotor.set(TalonSRXControlMode.PercentOutput, -0.3);
    }
    public void stop() {
        wristMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
