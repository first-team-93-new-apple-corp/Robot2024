package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    // State
    public enum States {
        notInit,
        Init,
        Run,
        Stop
    }

    public States State = States.notInit;

    // Subsystem
    public ElevatorSubsystem m_Elevator;

    // Local Variables
    public double setpoint = 0;

    public ElevatorCommand(ElevatorSubsystem m_ElevatorSubsystem) {
        m_Elevator = m_ElevatorSubsystem;
        if (State == States.notInit) {
            State = States.Init;
            m_Elevator.init();
        }
    }

    public void toAngle(double setpoint) {
        this.setpoint = setpoint;
    }

    public void stop() {
        State = States.Stop;
    }
    public void savePosition() {
        m_Elevator.savePosition();
    }
    @Override
    public void execute() {
        switch (State) {
            default:
            case Stop:
                m_Elevator.stop();
                break;
            case Run:
                m_Elevator.toSetpoint(setpoint);
                break;

            // The below values are not used, but are here in case
            case Init:
                break;
            case notInit:
                break;
        }
    }
}
