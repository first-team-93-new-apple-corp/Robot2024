package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Intake.IO.IntakeIOReal;
import frc.robot.subsystems.Intake.IO.IntakeIOSim;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class IntakeSubsystemFactory {
    public static IntakeSubsystem build(IntakeConstants constants, LEDSubsystem m_LedSubsystem, ShooterSubsystem m_ShooterSubsystem, XboxController op) {
        if (Utils.isSimulation()) {
            return new IntakeSubsystem(new IntakeIOSim(constants, m_LedSubsystem, m_ShooterSubsystem, op));
        } else {
            return new IntakeSubsystem(new IntakeIOReal(constants, m_LedSubsystem, m_ShooterSubsystem, op));
        }        
    }
}
