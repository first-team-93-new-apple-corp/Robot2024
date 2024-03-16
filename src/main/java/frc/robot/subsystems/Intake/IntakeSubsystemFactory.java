package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Intake.IO.IntakeIOReal;
import frc.robot.subsystems.Intake.IO.IntakeIOSim;

public class IntakeSubsystemFactory {
    public static IntakeSubsystem build(IntakeConstants constants, LEDSubsystem m_LedSubsystem, ShooterSubsystem m_ShooterSubsystem, XboxController op) {
        if (Utils.isSimulation()) {
            return new IntakeSubsystem(new IntakeIOSim(constants, m_LedSubsystem, m_ShooterSubsystem, op));
        } else {
            return new IntakeSubsystem(new IntakeIOReal(constants, m_LedSubsystem, m_ShooterSubsystem, op));
        }        
    }
}
