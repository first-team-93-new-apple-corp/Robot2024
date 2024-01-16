package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Interfaces.IGroundIntake;

public class GroundIntakeCmd extends CommandBase {
    private IGroundIntake m_IntakeSubsystem;
    Joystick js;
    
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public GroundIntakeCmd(IGroundIntake IntakeSubsystem) {
        this.m_IntakeSubsystem = IntakeSubsystem;
        addRequirements(m_IntakeSubsystem.asSubsystem());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (js.getRawButton(3)) { //TODO change with driver input
            m_IntakeSubsystem.intakeStart();
        }else {
            //Stop
           m_IntakeSubsystem.intakeStop(); 
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}