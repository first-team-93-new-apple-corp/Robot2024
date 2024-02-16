package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    XboxController op;
    ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    double setpoint = 0;

    public ElevatorCommand(XboxController op) {
        this.op = op;
    }
    public ElevatorCommand() {

    }
    public void initOnce() {
        m_elevator.initOnce();
    }
    public void preflight() {

    }

    @Override
    public void execute() {
        if (op.getPOV() == 0) {
            m_elevator.toSetpoint(-70);
        } else if (op.getPOV() == 90) {

        } else if (op.getPOV() == 180) {
            m_elevator.toSetpoint(-5);
        } else if (op.getPOV() == 270) {

        } else {
           
        }
    }
}
