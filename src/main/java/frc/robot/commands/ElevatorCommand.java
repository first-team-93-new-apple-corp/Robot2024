package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    XboxController op;
    ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    public ElevatorCommand(XboxController op) {
        this.op = op;
    }
    @Override
    public void execute() {
        if(op.getPOV(0) != -1) {
            m_elevator.runMotor(-0.1);
        } else if (op.getPOV(180) != -1) {
            m_elevator.runMotor(0.05);
        } else {
            m_elevator.runMotor(0);
        }
    }
}
