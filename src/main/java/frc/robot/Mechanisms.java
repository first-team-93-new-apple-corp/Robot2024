package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class Mechanisms {
    private double EleGearRatio = 20;
    private double ClimbGearRatio = 25;

    private Mechanism2d robotMechanism;

    private MechanismRoot2d elevatorMechanismRoot;
    private MechanismRoot2d climberMechanismRoot;

    private MechanismLigament2d elevatorMechanismLigament;
    private MechanismLigament2d climberMechanismLigament;

    private ElevatorSubsystem m_ElevatorSubsystem;
    private ClimberSubsystem m_ClimberSubsystem;

    //constructor
    public Mechanisms(ElevatorSubsystem m_ElevatorSubsystem, ClimberSubsystem m_ClimberSubsystem) {
        this.m_ElevatorSubsystem = m_ElevatorSubsystem;
        this.m_ClimberSubsystem = m_ClimberSubsystem;
        
        //CHANGE ALL VALUES WHEN UNDERSTAND GOODLY
        robotMechanism = new Mechanism2d(1.2, 1.2);
        elevatorMechanismRoot = robotMechanism.getRoot("Elevator", .51, .1);
        climberMechanismRoot = robotMechanism.getRoot("Climber", .33, .1);
        robotMechanism.getRoot("Robot", 0, .1).append(
            new MechanismLigament2d("frame", .86 , 0, 5, new Color8Bit(Color.kBlanchedAlmond))
        );

        elevatorMechanismLigament = elevatorMechanismRoot.append(
            new MechanismLigament2d("Elevator", .57, 120, 10, new Color8Bit(Color.kFirstRed))
        );

        climberMechanismLigament = climberMechanismRoot.append(
            new MechanismLigament2d("Climber", .39, 90, 10, new Color8Bit(Color.kFirstRed))
        );

    }

    public void periodic() {                //inital + Revolutions*gearRatio
        elevatorMechanismLigament.setLength(.57+ (m_ElevatorSubsystem.getPosition()/2048)*EleGearRatio);
        climberMechanismLigament.setLength(.39+(m_ClimberSubsystem.leftPosition()/2048)*ClimbGearRatio);
        
        SmartDashboard.putData("mech2d", robotMechanism);
    }
    
}