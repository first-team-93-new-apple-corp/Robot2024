package frc.robot.subsystems.Helpers;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM_EVENTS;
import frc.robot.Constants.ARM_SECTOR;
import frc.robot.Constants.ARM_SETPOINTS;
import frc.robot.Constants.ARM_STATES;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class ArmHelper extends SubsystemBase {
    ShoulderSubsystem m_ShoulderSubsystem;
    ElevatorSubsystem m_ElevatorSubsystem;
    public ARM_EVENTS currentEvent;
    private ARM_EVENTS[] armEvents;
    public ARM_SETPOINTS desiredSetpoint;
    private int eventIndex = 0;

    public ArmHelper(ShoulderSubsystem m_ShoulderSubsystem, ElevatorSubsystem m_ElevatorSubsystem) {
        this.m_ShoulderSubsystem = m_ShoulderSubsystem;
        this.m_ElevatorSubsystem = m_ElevatorSubsystem;
        currentEvent = ARM_EVENTS.END;
    }

    public void resetSubsystem(ARM_SETPOINTS desiredSetpoint) {
        this.desiredSetpoint = desiredSetpoint;

        eventIndex = 0;
        armEvents = getState().getEvents();

        currentEvent = armEvents[eventIndex];
    }

    public ARM_SECTOR getCurrentArmSector() {
        double shoulderPose = m_ShoulderSubsystem.getPostition();
        // double telescopePose = m_ElevatorSubsystem.getPostition();

        if (shoulderPose > 0.45) {
            return ARM_SECTOR.AMP;
        }

        else if (shoulderPose > 0) {
            return ARM_SECTOR.SHOOT;
        }

        else {
            return ARM_SECTOR.INTAKE;
        }
    }

    public ARM_STATES getState() {
        switch (getCurrentArmSector()) {
            case AMP:
                switch (desiredSetpoint.getDesiredArmSector()) {
                    case AMP:
                        return ARM_STATES.AMP_TO_AMP;

                    case INTAKE:
                        return ARM_STATES.AMP_TO_INTAKE;

                    case SHOOT:
                        return ARM_STATES.AMP_TO_SHOOT;

                    default:
                        return ARM_STATES.DO_NOTHING;
                }

            case SHOOT:
                switch (desiredSetpoint.getDesiredArmSector()) {
                    case AMP:
                        return ARM_STATES.SHOOT_TO_AMP;

                    case INTAKE:
                        return ARM_STATES.SHOOT_TO_INTAKE;

                    case SHOOT:
                        return ARM_STATES.SHOOT_TO_SHOOT;

                    default:
                        return ARM_STATES.DO_NOTHING;
                }

            case INTAKE:
                switch (desiredSetpoint.getDesiredArmSector()) {
                    case AMP:
                        return ARM_STATES.INTAKE_TO_AMP;

                    case INTAKE:
                        return ARM_STATES.INTAKE_TO_INTAKE;

                    case SHOOT:
                        return ARM_STATES.INTAKE_TO_SHOOT;

                    default:
                        return ARM_STATES.DO_NOTHING;
                }

            default:
                return ARM_STATES.DO_NOTHING;
        }
    }

    public Subsystem[] getRequiredSubsystems() {
        return new Subsystem[] {
                m_ShoulderSubsystem,
                m_ElevatorSubsystem
        };
    }

    public void allToSetpoint(ARM_SETPOINTS armSetpoint) {
        m_ShoulderSubsystem.toSetpoint(armSetpoint.getShoulderPosition());
        m_ElevatorSubsystem.toSetpoint(armSetpoint.getElevatorPosition());
    }

    public void shoulderToSetpoint() {
        m_ShoulderSubsystem.toSetpoint(desiredSetpoint.getShoulderPosition());
    }

    public void retractTelescope() {
        m_ElevatorSubsystem.toSetpoint(desiredSetpoint.getElevatorPosition());
    }

    public void extendTelescope() {
        m_ElevatorSubsystem.toSetpoint(desiredSetpoint.getElevatorPosition());
    }

    public void shoulderToSetpointNC() {
        m_ShoulderSubsystem.toSetpoint(desiredSetpoint.getShoulderPosition());
        m_ElevatorSubsystem.toSetpoint(desiredSetpoint.getElevatorPosition());
    }

    public void follow() {
        switch (currentEvent) {
            case END:
            default:
                break;

            case ALL_TO_SP:
                allToSetpoint(null);
                break;

            case SHOULDER_TO_SP:
                shoulderToSetpoint();
                break;

            case RETRACT_TELESCOPE:
                retractTelescope();
                break;

            case EXTEND_TELESCOPE:
                extendTelescope();
                break;

            case SHOULDER_TO_SP_NC:
                shoulderToSetpointNC();
                break;
        }
        if (allAtSetpoint() && eventIndex < armEvents.length - 1) {
            switchToNextEvent();
        }

    }

    public boolean allAtSetpoint() {
        return m_ShoulderSubsystem.atSetpoint() && m_ElevatorSubsystem.atSetpoint();
    }

    public void switchToNextEvent() {
        eventIndex++;
        currentEvent = armEvents[eventIndex];
    }
}