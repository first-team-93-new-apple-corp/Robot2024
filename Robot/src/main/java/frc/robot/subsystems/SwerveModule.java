package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  double Motor_Commands;
  double Turning_Degrees;

  WPI_TalonFX Driving_Motor;
  WPI_TalonFX Turning_Motor;
  WPI_CANCoder Can_Coder;

  AbsoluteSensorRange Range;

  int driveMotorID;

  public SwerveModule(
    int driveMotorID,
    int turnMotorID,
    int CanCoderID,
    double magnetOffset
  ) {

    this.driveMotorID = driveMotorID;

    Driving_Motor = new WPI_TalonFX(driveMotorID);
    Driving_Motor.setNeutralMode(NeutralMode.Brake);
    Driving_Motor.setInverted(true);
    SmartDashboard.putString("State " + driveMotorID, "test");
    

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    // driveConfig.supplyCurrLimit.enable = true;
    // driveConfig.supplyCurrLimit.currentLimit = 5;
    // driveConfig.supplyCurrLimit.triggerThresholdCurrent = 5;
    // driveConfig.supplyCurrLimit.triggerThresholdTime = .254;

    Driving_Motor.configFactoryDefault();
    Driving_Motor.configAllSettings(driveConfig);

    Turning_Motor = new WPI_TalonFX(turnMotorID);
    Turning_Motor.setNeutralMode(NeutralMode.Brake);


    Turning_Motor.configFactoryDefault();
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    // turnConfig.supplyCurrLimit.enable = true;
    // turnConfig.supplyCurrLimit.currentLimit = 5;
    // turnConfig.supplyCurrLimit.triggerThresholdCurrent = 5;
    // turnConfig.supplyCurrLimit.triggerThresholdTime = .254;
    turnConfig.slot0.kP = DriveConstants.Turning_P;
    turnConfig.slot0.kI = DriveConstants.Turning_I;
    turnConfig.slot0.kD = DriveConstants.Turning_D;
    turnConfig.slot0.allowableClosedloopError =
      DriveConstants.Turning_Tolerance;

    Turning_Motor.configAllSettings(turnConfig);

    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);

    Can_Coder = new WPI_CANCoder(CanCoderID);
    Can_Coder.configMagnetOffset(magnetOffset);
    Range = AbsoluteSensorRange.valueOf(0);
    Can_Coder.configAbsoluteSensorRange(Range);
    Turning_Motor.setSelectedSensorPosition(
      (Can_Coder.getAbsolutePosition() / 360.) * (2048 * 12.8)
    );
    Driving_Motor.setSelectedSensorPosition(0.0);
  }

  SwerveModuleState state = new SwerveModuleState(0, new Rotation2d());

  public void setDesiredState(SwerveModuleState desiredState) {
    double currentPos = Turning_Motor.getSelectedSensorPosition();
    state =
      SwerveModuleState.optimize(
        desiredState,
        Rotation2d.fromDegrees(Can_Coder.getAbsolutePosition())
      );

    double errorBound = (360) / 2.0;
    double m_positionError = MathUtil.inputModulus(
      state.angle.getDegrees() - (ticksToDegrees(currentPos)),
      -errorBound,
      errorBound
    );


    // do not need PID on drive motors - just a simple voltage calculation
    double driveOutput =
      (state.speedMetersPerSecond / DriveConstants.Max_Strafe_Speed) *
      DriveConstants.Max_Volts;

    Driving_Motor.setVoltage(driveOutput);

    Turning_Motor.set(
      ControlMode.Position,
      currentPos + degreesToTicks(m_positionError)
    );
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDistance(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public double radsToTicks(double radians) {
    return radians / (2 * Math.PI / (12.8 * 2048));
  }

  public static double degreesToTicks(double degrees) {
    return (degrees / 360) * (12.8 * 2048);
  }

  public static double ticksToDegrees(double ticks) {
    return ((ticks * 360) / (12.8 * 2048)) % 360;
  }

  int counter;

  // calculate the current velocity of the driving wheel
  public double getVelocity() {
    double VelocityInTicks = Driving_Motor.getSelectedSensorVelocity() * 10;
    double speed =
      VelocityInTicks * Units.inchesToMeters(4 * Math.PI) / (6.75 * 2048);
    counter++;
    if (counter % 50 == 0) {
    }
    return speed;
  }

  public double getDistance() {
    return Units.inchesToMeters(Driving_Motor.getSelectedSensorPosition() * 4 * Math.PI) / (DriveConstants.Driving_Gearing * 2048);
  }

  // get angle from can coder
  // test to see if get absolute position is continuous
  public Rotation2d getAngle() {
    // return Rotation2d.fromDegrees((Can_Coder.getAbsolutePosition()));
    return Rotation2d.fromDegrees(ticksToDegrees(Turning_Motor.getSelectedSensorPosition()));
  }

  public double getCancoderTicks(){
    return Can_Coder.getAbsolutePosition(); 
  }
  public double FrontRightRotations(){
    return Driving_Motor.getSelectedSensorPosition()/(2048*DriveConstants.Driving_Gearing);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("State " + driveMotorID, getState().toString());
    SmartDashboard.putNumber("FrontRightRotations", FrontRightRotations());
  }

  @Override
  public void simulationPeriodic() {}
}