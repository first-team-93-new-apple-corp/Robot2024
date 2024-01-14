package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements IGroundIntake{
    
    @Override
    public void intakeStart() {
        System.out.println("Ground Intaking");
    }

    @Override
    public void intakeStop() {
        System.out.println("Brake Ground Motors");
    }

    @Override
    public SubsystemBase asSubsystem() {
        return this;
    }
    
}
