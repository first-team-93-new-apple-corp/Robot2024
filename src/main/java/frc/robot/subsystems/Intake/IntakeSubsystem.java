package frc.robot.subsystems.Intake;


import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Intake.IO.IntakeIO;
import frc.robot.subsystems.Intake.IO.IntakeIO.IntakeIOInputs;;

public class IntakeSubsystem extends SubsystemBase {
    
    private final IntakeIO m_io;
    private IntakeIOInputs m_Inputs = new IntakeIOInputs();

    public IntakeSubsystem(IntakeIO io) {
        m_io = io;
    }

    public void intake() {
        m_io.Intake();
    }
    
    public void resetIntakeState() {
        m_io.resetIntakeState();
    }


    public void stop() {
        m_io.stop();
    }

    public void passthrough(){
        m_io.passthrough();
    }
    
    @Override
    public void periodic() {
        m_io.updateValues(m_Inputs);
        SignalLogger.writeBoolean("Intake:Currently Intaking", m_Inputs.CrrentlyIntaking);
        SignalLogger.writeBoolean("Intake:Note In Intake", m_Inputs.NoteInIntake);
    }
}