package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Interfaces.IGroundIntake;

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
