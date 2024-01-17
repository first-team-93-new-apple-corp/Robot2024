package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Interfaces.IGroundIntake;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase implements IGroundIntake{
    CANSparkMax intakeR = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax intakeL = new CANSparkMax(5, MotorType.kBrushless);
    final double intakeSetpoint = 0.75;
    @Override
    public void intakeStart() {
        System.out.println("Ground Intaking");
        intakeL.set(-intakeSetpoint);
        intakeR.set(intakeSetpoint);
    }

    @Override
    public void intakeRunover(){
        intakeL.set(intakeSetpoint);
        intakeR.set(intakeSetpoint);
    }

    @Override
    public void intakeStop() {
        System.out.println("Brake Ground Motors");
        intakeL.setIdleMode(IdleMode.kBrake);
        intakeR.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public SubsystemBase asSubsystem() {
        return this;
    }
    
}
