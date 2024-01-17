package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Interfaces.IGroundIntake;

public class GroundIntakeCmd extends CommandBase {
    private IGroundIntake m_IntakeSubsystem;
    private Joystick js;
    private XboxController xjs;
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
        if (js.getRawButton(3) || xjs.getRawButton(Constants.F310D.X)) { //TODO change with driver input
            m_IntakeSubsystem.intakeStart();
        } else if (js.getRawButton(4) || xjs.getRawButton(Constants.F310D.A)) {
            m_IntakeSubsystem.intakeRunover();
        } else {
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