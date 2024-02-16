package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    XboxController op;
    ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    double setpoint = 0;
    double ampSetpoint;
    TalonFX m_motor = new TalonFX(Constants.CTRE.RIO.Elevator, "rio");
    double currentPos = m_motor.getPosition().getValueAsDouble();

    public ElevatorCommand(XboxController op) {
        this.op = op;
    }
    public void initOnce() {
        m_elevator.initOnce();
    }
    public void preflight() {
        m_elevator.zero();
    }

    @Override
    public void execute() {
        if (op.getPOV() == 0) {
            m_elevator.toSetpoint(-75);
        } else if (op.getPOV() == 90) {
            m_elevator.toSetpoint(currentPos += 5);
        } else if (op.getPOV() == 180) {
            m_elevator.toSetpoint(-3);
        } else if (op.getPOV() == 270) {
            m_elevator.toSetpoint(currentPos -= 5);
        } else {
           
        }
    }
}
