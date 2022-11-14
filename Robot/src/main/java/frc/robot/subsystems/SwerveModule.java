package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  double Motor_Commands;
  double Turning_Degrees;

  AHRS Gyro;

  WPI_TalonFX Driving_Motor;
  WPI_TalonFX Turning_Motor;
  WPI_CANCoder Can_Coder;
  AbsoluteSensorRange Range;
  double LastAngle;
  SimpleMotorFeedforward feedForward;

  PIDController TurningPID = new PIDController(
    DriveConstants.Turning_P,
    DriveConstants.Turning_I,
    DriveConstants.Turning_D
  );

  ProfiledPIDController TurningProfiledPID = new ProfiledPIDController(
    DriveConstants.Turning_P,
    DriveConstants.Turning_I,
    DriveConstants.Turning_D,
    new Constraints(1, 6.28)
  );

  public SwerveModule(
    int driveMotorID,
    int turnMotorID,
    int CanCoderID,
    double magnetOffset
  ) {
    feedForward = new SimpleMotorFeedforward(0.6099, 0.216);
    Driving_Motor = new WPI_TalonFX(driveMotorID);
    Driving_Motor.setNeutralMode(NeutralMode.Brake);
    Driving_Motor.setInverted(true);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.supplyCurrLimit.enable = true;
    driveConfig.supplyCurrLimit.currentLimit = 5;
    driveConfig.supplyCurrLimit.triggerThresholdCurrent = 5;
    driveConfig.supplyCurrLimit.triggerThresholdTime = .254;

    Driving_Motor.configFactoryDefault(); 
    Driving_Motor.configAllSettings(driveConfig);

    
    Turning_Motor = new WPI_TalonFX(turnMotorID);
    Turning_Motor.setNeutralMode(NeutralMode.Brake);
    


    /*This is Nolen test code no touchy */
    Turning_Motor.configFactoryDefault();
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.supplyCurrLimit.enable = true;
    turnConfig.supplyCurrLimit.currentLimit = 5;
    turnConfig.supplyCurrLimit.triggerThresholdCurrent = 5;
    turnConfig.supplyCurrLimit.triggerThresholdTime = .254;
    Turning_Motor.configAllSettings(turnConfig);

    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
  

    Can_Coder = new WPI_CANCoder(CanCoderID);
    Can_Coder.configMagnetOffset(magnetOffset);
    Range = AbsoluteSensorRange.valueOf(0);
    Can_Coder.configAbsoluteSensorRange(Range);


    TurningPID.setTolerance(DriveConstants.Turning_Tolerance);
    TurningPID.enableContinuousInput(-Math.PI, Math.PI);
    TurningProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
  }



  public void setDesiredState(SwerveModuleState desiredState) {
    // if 0,0,0 is in here after auto what happens?

    // optimize which way to turn the wheel
    SwerveModuleState state = SwerveModuleState.optimize(
      desiredState,
      getAngle()
    );

    //do not need PID on drive motors - just a simple voltage calculation
    double driveOutput =
      (state.speedMetersPerSecond / DriveConstants.Max_Strafe_Speed) *
      DriveConstants.Max_Volts;

    //Turning needs a pid because it has a setpoint it need to reach
    double turnOutput = TurningPID.calculate(
      getAngle().getRadians(),
      state.angle.getRadians()
      //why doesn't optimize or this fix this if states aren't recorded
    );

 

    // double feedOutput = feedforward.calculate(turnOutput/12.0*6.28);

    Driving_Motor.setVoltage(driveOutput);
    // state.angle = new Rotation2d(2);
    Turning_Motor.setVoltage(turnOutput);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public double radsToTicks(double radians){
    // return radians / (2*Math.PI / (6.28 * 2048)); 
    return radians * 2048.;
  }

  // public void resetEncoders() { //need to figure out offsets
  //   Driving_Motor.setSelectedSensorPosition(0);
  //   Turning_Motor.setSelectedSensorPosition(0);
  // }

  // calculate the current velocity of the driving wheel
  public double getVelocity() {
    double speed =
      Driving_Motor.getSelectedSensorVelocity() *
      10 /
      DriveConstants.TalonFX_Encoder_Resolution /
      DriveConstants.Driving_Gearing *
      DriveConstants.Wheel_Circumference;
    return Units.feetToMeters(speed);
  }

  // get angle from can coder
  //test to see if get absolute position is continuous
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Can_Coder.getAbsolutePosition());
  }

  @Override
  public void periodic() {

   DriveConstants.Turning_P = SmartDashboard.getNumber("P", 0);
   DriveConstants.Turning_I = SmartDashboard.getNumber("I", 0);
   DriveConstants.Turning_D =  SmartDashboard.getNumber("D", 0);
    // System.out.println(MathUtil.angleModulus(100000));
    //this for example does wrap the angle
// System.out.println(getAngle());
  }

  @Override
  public void simulationPeriodic() {}
}
