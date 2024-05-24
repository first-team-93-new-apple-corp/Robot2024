package frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.IO.ElevatorIO;
import frc.robot.subsystems.Elevator.IO.ElevatorIO.ElevatorIOInputs;

public class ElevatorSubsystem extends SubsystemBase {
    
    private final ElevatorIO m_io;
    private ElevatorIOInputs m_Inputs = new ElevatorIOInputs();

    public ElevatorSubsystem(ElevatorIO io) {
        m_io = io;
    }

    public void disable(){
        m_io.disable();
    }

    public void initOnce(){
        m_io.initOnce();
    }

    public boolean topLimitTriggered(){
        return m_io.topLimitTriggered();
    }

    public boolean bottomLimitTriggered(){
        return m_io.bottomLimitTriggered();
    }

    public void runElevator(){
        m_io.runElevator();
    }
    public boolean atSetpoint(){
        return m_io.atSetpoint();
    }
    public void stopElevator(){
        m_io.stopElevator();
    }
    public void toSetpoint(double newSetpoint){
        m_io.toSetpoint(newSetpoint);
    }
    public double getPosition(){
        return m_io.ElevatorPosition();
    }

    public void zero(){
        m_io.zero();
    }

    public DoubleSupplier ElevatorPostion = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return getPosition();
        }
    };
    @Override
    public void periodic() {
        m_io.updateValues(m_Inputs);
        SignalLogger.writeDouble("Elevator:Current Positiom", m_io.ElevatorPosition());
        SignalLogger.writeBoolean("Elevator:Top Limit Triggered", topLimitTriggered());
        SignalLogger.writeBoolean("Elevator:Bottom Limit Triggered", bottomLimitTriggered());

        SmartDashboard.putNumber("Elevator Pos", m_io.ElevatorPosition());
        SmartDashboard.putBoolean("Top Limit?", topLimitTriggered());
        SmartDashboard.putBoolean("Bottom Limit?", bottomLimitTriggered());
    }
}