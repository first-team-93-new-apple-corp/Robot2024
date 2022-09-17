// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class TestSysIDPosHold extends SubsystemBase {
//   SimpleMotorFeedforward m_feedforward;
//   PIDController m_pidController;
//   TrapezoidProfile Trap;
//   TrapezoidProfile.Constraints Constraints = new TrapezoidProfile.Constraints(1,1);
//   ProfiledPIDController m_profiledPIDController;
//   WPI_TalonFX Motor;
//   public TestSysIDPosHold() {

//     m_feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
//     m_pidController = new PIDController(0.0, 0.0, 0.0);
//     m_profiledPIDController = new ProfiledPIDController(m_pidController.getP(), m_pidController.getI(), m_pidController.getD(), Constraints);
//     Motor = new WPI_TalonFX(0);
   
//   }

//   public void GotoSetpoint() {
//     double output = m_feedforward.calculate(m_profiledPIDController.getGoal().velocity); //this is the output of the feedforward 
//     double output2 = m_profiledPIDController.calculate(getRadians()); //this is the output of the profiled PID controller
//     Motor.setVoltage(output + output2);
//   }
//   public void SetSetpoint(double PositionSetpoint){
//     m_profiledPIDController.setGoal(PositionSetpoint);
//   }

//   public double getRadians(){
//     return 0;
//   }
//   public void Set(double speed){

//   }

//   @Override
//   public void periodic() {}

//   @Override
//   public void simulationPeriodic() {}
// }