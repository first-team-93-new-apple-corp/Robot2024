package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;
class ShoulderCommand extends Command {
    public ShoulderSubsystem m_shoulder;
    public double angleSetpoint = 0;
    public enum States {
        init,notInit,Run,Stop
    }

    public States shoulderState = States.notInit;

    public void setSetpoint(double setpoint) {
        angleSetpoint = setpoint;
    }

    public ShoulderCommand(ShoulderSubsystem m_shoulder) {
        this.m_shoulder = m_shoulder;
        if (shoulderState.equals(States.notInit)) {
            shoulderState = States.init;
            
        }
    }

    public Command moveShoulder() {
        return (m_shoulder.run(() -> m_shoulder.moveShoulder(angleSetpoint)));
    }

    public Command stopShoulder() {
        return (m_shoulder.runOnce(() -> m_shoulder.stopShoulder()));
    }
}