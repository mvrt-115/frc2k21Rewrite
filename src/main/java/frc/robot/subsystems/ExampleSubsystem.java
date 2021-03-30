// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;

public class ExampleSubsystem extends SubsystemBase {

  public static enum ExampleSubsystemState {
    RUNNING, DISABLED
  }

  private ExampleSubsystemState currentState;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    // initialize Hardware components
    Hardware.ExampleSubsystem.rightLeader = new WPI_TalonSRX(1);

    Hardware.ExampleSubsystem.rightLeader.configFactoryDefault();

    setCurrentState(ExampleSubsystemState.DISABLED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(currentState){
      case RUNNING:
        runMotors();
        break;
      case DISABLED:
        stopMotors();
        break;
    }
    log();
  }

  public void log(){
    SmartDashboard.putString("ExampleSubsystemState", currentState.name());
    SmartDashboard.putNumber("ExampleSubsystem rightLeader", Hardware.ExampleSubsystem.rightLeader.getMotorOutputVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setCurrentState(ExampleSubsystemState newState){
    currentState = newState;
  }

  public void runMotors(){
    Hardware.ExampleSubsystem.rightLeader.set(ControlMode.PercentOutput, Constants.ExampleSubsystem.SPEED);
  }
  
  public void stopMotors(){
    Hardware.ExampleSubsystem.rightLeader.set(ControlMode.PercentOutput, 0);
  }
}
