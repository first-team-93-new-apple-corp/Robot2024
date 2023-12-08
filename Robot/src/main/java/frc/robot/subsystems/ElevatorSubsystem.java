// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {
  PIDController elevatorPID = new PIDController(0.001, 0 , 0.0005);
  int[] setpoint = new int[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
  int mass;
  DigitalInput limitSwitch = new DigitalInput(0);
  int currentsetpoint;
  int currentHeight;
  int maxHeight;
  int minHeight;
  int gearRatio = 25; // Change later!??!?!?
  boolean driverControl = false;
  boolean run = true;
  WPI_TalonFX ElevMotor = new WPI_TalonFX(01);
  Joystick ElevJoystick = new Joystick(0);
  ElevatorFeedforward ElevFeedforward = new ElevatorFeedforward(maxHeight, 0.21, 24.47, 0.03);

  public ElevatorSubsystem() {
  }

  // public void ElevMain() {
  // // toggle
  // // if (ElevJoystick.getRawButtonPressed(1)) {
  // // driverControl = !driverControl;
  // // }
  // // if (driverControl) {
  // // JoystickControl();
  // // } else {
  // // ButtonControl();
  // // }

  // // while held
  // if (ElevJoystick.getRawButton(1)) {
  // JoystickControl();
  // } else {
  // ButtonControl();
  // }
  // }

  public void ButtonControl() {
    // Change buttons later
    for (int i = 5; i < 17; i++) {
      if (ElevJoystick.getRawButton(i)) {
        currentsetpoint = setpoint[i - 5];
        goToSetpoint();
        return;
      }
      // Making it how nagle wants it so that when a button is not pressed we can
      // drive stick (if a button is pressed it returns to run it again if not we get
      // to drive)
      JoystickControl();
    }
  }

  public void goToSetpoint() {
    ElevMotor.set(ElevFeedforward.calculate(elevatorPID.calculate(ElevMotor.getSelectedSensorPosition(), currentsetpoint)));
  }

  public void JoystickControl() {
    if (((ElevMotor.getSelectedSensorPosition() > maxHeight) && (ElevJoystick.getY() > 0))
        || ((ElevMotor.getSelectedSensorPosition() < minHeight) && (ElevJoystick.getY() < 0))) {
      return;
    } else if (limitSwitch.get() && ElevJoystick.getY() < 0) {
      ElevMotor.set(0);
    } else {
      ElevMotor.set((ElevJoystick.getY()) * (ElevJoystick.getY()));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
