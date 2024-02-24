package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Preflight extends Command {
    PowerDistribution pdh;
    ClimberCommand climbers;
    ElevatorCommand elevatorCommand;
    IntakeCommand m_IntakeCommand;
    OrchestraCommand m_OrchestraCommand;
    SlewRateLimiter left = new SlewRateLimiter(0.5, -0.5, 0.5);
    SlewRateLimiter right = new SlewRateLimiter(0.5, -0.5, 0.5);
    private static boolean leftFinished = false;
    private static boolean rightFinished = false;
    private static boolean elevatorFinished = false;

    public Preflight(XboxController op, ElevatorCommand elevatorCommand, IntakeCommand m_IntakeCommand, ClimberCommand climbers) {
        pdh = new PowerDistribution(1, ModuleType.kRev);
        this.climbers = climbers;
        this.elevatorCommand = elevatorCommand;
        this.m_IntakeCommand = m_IntakeCommand;
        m_OrchestraCommand = new OrchestraCommand();
    }
    public static boolean isPreflightDone() {
        return (leftFinished && rightFinished && elevatorFinished);
    }
    public void resetPreflight() {
        leftFinished = false;
        rightFinished = false;
        elevatorFinished = false;
    }

    @Override
    public void execute() {
        if (!leftFinished) {
            if (!(left.calculate(climbers.leftDraw()) > 0.6)) {
                climbers.windLeft(0.05);
            } else {
                climbers.windLeft(0);
                climbers.zeroLeft();
                leftFinished = true;
            }
        }
        if (!rightFinished) {
            if (!(right.calculate(climbers.rightDraw()) > 0.6)) {
                climbers.windRight(0.05);
            } else {
                climbers.windRight(0);
                climbers.zeroRight();
                rightFinished = true;
            }
        }
        if(!elevatorFinished) {
          elevatorCommand.preflight();  
          elevatorFinished = true;
        } 
        
        // elevatorCommand.preflight();
        if (leftFinished && rightFinished) {
            SmartDashboard.putBoolean("Preflight Done?", true);
            m_OrchestraCommand.play();
        }
    }
}