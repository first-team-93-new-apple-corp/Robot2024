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
    @Override
    public void execute() {
        
        // if (op.getPOV() == 0) {
        //     // m_elevatorSubsystem.toSetpoint(-75);
        //     setpoint -= 5;
        // } else if (op.getPOV() == 90) {
        //     // m_elevatorSubsystem.toSetpoint(-40);
        // } else if (op.getPOV() == 180) {
        // // m_elevatorSubsystem.toSetpoint(-3);
        //     setpoint += 5;
        // } else if (op.getPOV() == 270) {
        //     // m_elevatorSubsystem.toSetpoint(-25);
        // }

        // m_elevatorSubsystem.toSetpoint(setpoint);

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
