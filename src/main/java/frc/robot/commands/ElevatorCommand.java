package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    XboxController op;
    ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    double setpoint = 0;
    double ampSetpoint = -50;
    double sourceSetpoint = -3;
    private Servo ampServo = new Servo(8);
    

    public ElevatorCommand(XboxController op) {
        this.op = op;
    }

    public void initOnce() {
        m_elevator.initOnce();
    }
    public boolean done() {
        return m_elevator.bottomLimitTriggered();
    }
    public void preflight() {
        m_elevator.zero();
    }
    public void runElevator() {
        m_elevator.runElevator();
    }

    public TalonFX getMotor() {
        return m_elevator.getMotor();
    }

    public void disable() {
        m_elevator.disable();
    }
    public void ServoUp() {
        ampServo.set(0.1);
    }

    public void ServoDown() {
        ampServo.set(0.65);
    }
    @Override
    public void execute() {
        
        // if (op.getPOV() == 0) {
        //     // m_elevator.toSetpoint(-75);
        //     setpoint -= 5;
        // } else if (op.getPOV() == 90) {
        //     // m_elevator.toSetpoint(-40);
        // } else if (op.getPOV() == 180) {
        // // m_elevator.toSetpoint(-3);
        //     setpoint += 5;
        // } else if (op.getPOV() == 270) {
        //     // m_elevator.toSetpoint(-25);
        // }

        // m_elevator.toSetpoint(setpoint);

        if (op.getRawButton(Constants.xbox.RightShoulderButton)) {
            m_elevator.toSetpoint(ampSetpoint);
            this.ServoUp();
        } else if (op.getRawButton(Constants.xbox.LeftShoulderButton)) {
            m_elevator.toSetpoint(sourceSetpoint);
        } else {
            m_elevator.toSetpoint(3);
            this.ServoDown();
        }
        m_elevator.runElevator();
    }
}
