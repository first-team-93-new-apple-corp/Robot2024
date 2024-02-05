package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    XboxController op;
    ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    double upSpeed = -0.5;
    double downSpeed = 0.2;
    public ElevatorCommand(XboxController op) {
        this.op = op;
    }
    @Override
    public void execute() {
        if(op.getPOV() == 0) {
            m_elevator.runMotor(upSpeed);
        } else if (op.getPOV() == 180) {
            m_elevator.runMotor(downSpeed);
        } else {
            m_elevator.runMotor(0);
        }
    }
}
