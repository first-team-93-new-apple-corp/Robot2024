package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    XboxController op;
    ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    double upSpeed = -01;
    double downSpeed = 0.6;
    double setpoint = 0;

    public ElevatorCommand(XboxController op) {
        this.op = op;
    }

    public void initOnce() {
        m_elevator.initOnce();
    }

    @Override
    public void execute() {
        if (op.getPOV() == 0) {
            // m_elevator.runMotor(upSpeed);
            setpoint -= 5;
        } else if (op.getPOV() == 180) {
            // m_elevator.runMotor(downSpeed);
            setpoint += 5;
        } else {
            // m_elevator.runMotor(0);
        }
        if (setpoint > 0) {
            setpoint = 0;
        }
        if (setpoint < -75) {
            setpoint = -75;
        }
        m_elevator.toSetpoint(setpoint);
    }
}
