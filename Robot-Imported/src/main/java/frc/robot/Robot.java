package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveModule;


public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public static SwerveModule Front_Left;
  public static SwerveModule Front_Right;
  public static SwerveModule Back_Left;
  public static SwerveModule Back_Right;
  @Override
  public void robotInit() {
    setNetworkTablesFlushEnabled(true);
    m_robotContainer = new RobotContainer();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.m_TeleopDriveCommand.disableLights();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

    SmartDashboard.putBoolean("Enabled?", isEnabled());

  }

  @Override
  public void disabledInit() {
    m_robotContainer.m_TeleopDriveCommand.disableLights();
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.scheduleTeleopCommands();
    m_robotContainer.setTeleopBindings();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

    
  }
}
