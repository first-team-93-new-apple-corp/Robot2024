package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    XboxController op;
    ElevatorSubsystem m_elevatorSubsystem;
    double setpoint = 0;
    double ampSetpoint = -50;
    double sourceSetpoint = -70;

    public ElevatorCommand(XboxController op, ElevatorSubsystem m_ElevatorSubsystem) {
        this.op = op;
        this.m_elevatorSubsystem = m_ElevatorSubsystem;
    }

    public void initOnce() {
        m_elevatorSubsystem.initOnce();
    }

    public void preflight() {
        m_elevatorSubsystem.zero();
    }


    public void disable() {
        m_elevatorSubsystem.disable();
    }

    
    public Command stopElevator() {
        return m_elevatorSubsystem.runOnce(m_elevatorSubsystem::stopElevator);
    }

    public Command runElevator() {
        return m_elevatorSubsystem.runOnce(m_elevatorSubsystem::runElevator);
    }

    public Command Amp(){
        return (m_elevatorSubsystem.runOnce(()-> m_elevatorSubsystem.toSetpoint(ampSetpoint)))
        .andThen(this.runElevator().repeatedly().until(m_elevatorSubsystem::atSetpoint))
        .andThen(stopElevator());
    }

    public Command Source(){
        return (m_elevatorSubsystem.runOnce(()-> m_elevatorSubsystem.toSetpoint(sourceSetpoint)))
        .andThen(this.runElevator().repeatedly().until(m_elevatorSubsystem::atSetpoint))
        .andThen(stopElevator());
    }   

    public Command Default(){
        return (m_elevatorSubsystem.runOnce(()-> m_elevatorSubsystem.toSetpoint(3)))
        .andThen(this.runElevator().repeatedly().until(m_elevatorSubsystem::atSetpoint))
        .andThen(stopElevator());
    }

    @Override
    public void execute() {
        
        if (op.getRawButton(Constants.xbox.RightShoulderButton)) {
            m_elevatorSubsystem.toSetpoint(ampSetpoint);
        } else if (op.getRawButton(Constants.xbox.LeftShoulderButton)) {
            m_elevatorSubsystem.toSetpoint(sourceSetpoint);
        } else {
            m_elevatorSubsystem.toSetpoint(3);
        }
        m_elevatorSubsystem.runElevator();
    }
}
