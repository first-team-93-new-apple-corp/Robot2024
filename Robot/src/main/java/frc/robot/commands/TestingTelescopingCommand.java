package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingSubsystem;

public class TestingTelescopingCommand extends CommandBase {

    TelescopingSubsystem m_TelescopingSubsystem;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public TestingTelescopingCommand(TelescopingSubsystem m_TelescopingSubsystem) {
        this.m_TelescopingSubsystem = m_TelescopingSubsystem;

        addRequirements(m_TelescopingSubsystem);

        SmartDashboard.putNumber("Arm Setpoint", 0);
    }

    @Override
    public void initialize() {

    }

    enum ArmState {
        ZEROING,
        STANBY,
        SETPOINT

    }

    ArmState currentArmState = ArmState.ZEROING;

    @Override
    public void execute() {
        // m_TelescopingSubsystem.OscilateArm();
        // m_TelescopingSubsystem.directMotorCommand(0.1);

        switch (currentArmState) {
            case STANBY:
                if (SmartDashboard.getNumber("Arm Setpoint", -1) != 0) {
                    currentArmState = ArmState.SETPOINT;
                }
                break;
            case SETPOINT:
                m_TelescopingSubsystem.toSetpoint(SmartDashboard.getNumber("Arm Setpoint", 0));

                break;
            case ZEROING:
                if (!(m_TelescopingSubsystem.getTicks() <= 140)) {
                    m_TelescopingSubsystem.directMotorCommand(-0.1);
                } else {
                    m_TelescopingSubsystem.directMotorCommand(0);
                    currentArmState = ArmState.STANBY;
                }
                break;
            default:
                break;

        }

    }

    @Override
    public void end(boolean interrupted) {
        m_TelescopingSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
