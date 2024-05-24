package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mechanisms extends SubsystemBase{
    private double EleGearRatio = -10;
    private double ClimbGearRatio = -5;

    private Mechanism2d robotMechanism;

    private MechanismRoot2d elevatorMechanismRoot;
    private MechanismRoot2d climberMechanismRoot;

    private MechanismLigament2d elevatorMechanismLigament;
    private MechanismLigament2d climberMechanismLigament;

    private DoubleSupplier m_ElevatorPosition;
    private DoubleSupplier m_ClimberPostion;

    //constructor
    public Mechanisms(DoubleSupplier m_ElevatorPosition, DoubleSupplier m_ClimberPostion) {
        this.m_ElevatorPosition = m_ElevatorPosition;
        this.m_ClimberPostion = m_ClimberPostion;
        
        //CHANGE ALL VALUES WHEN UNDERSTAND GOODLY
        robotMechanism = new Mechanism2d(1.2, 1.2);
        elevatorMechanismRoot = robotMechanism.getRoot("Elevator", .37, .1);
        climberMechanismRoot = robotMechanism.getRoot("Climber", .53, .1);
        robotMechanism.getRoot("Robot", 0, .1).append(
            new MechanismLigament2d("frame", .86 , 0, 5, new Color8Bit(Color.kBlanchedAlmond))
        );

        elevatorMechanismLigament = elevatorMechanismRoot.append(
            new MechanismLigament2d("Elevator", .57, 60, 3, new Color8Bit(Color.kFirstRed))
        );

        climberMechanismLigament = climberMechanismRoot.append(
            new MechanismLigament2d("Climber", .39, 90, 3, new Color8Bit(Color.kFirstRed))
        );

    }

    @Override
    public void periodic() {                //inital + Revolutions*gearRatio
        elevatorMechanismLigament.setLength(.57+ (m_ElevatorPosition.getAsDouble()/2048)*EleGearRatio);
        climberMechanismLigament.setLength(.39+(m_ClimberPostion.getAsDouble()/2048)*ClimbGearRatio);
        
        SmartDashboard.putData("mech2d", robotMechanism);
    }
    
}