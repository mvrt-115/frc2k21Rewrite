// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

/** 
 * The hopper subsystem brings the balls from the intake
 * to the flywheel. The hopper is controlled by 2 parts, 
 * the top and bottom.
 */
public class Hopper extends SubsystemBase {

  private State state;

  public static enum State {
    RUNNING,
    DISABLED
  }

  /** Creates a new Hopper. */
  public Hopper() {
    if(RobotBase.isReal()) {
      Hardware.Hopper.top = new TalonFX(0);
      Hardware.Hopper.bottom = new TalonFX(1);
    } else {
      Hardware.Hopper.top = new WPI_TalonSRX(0);
      Hardware.Hopper.bottom = new WPI_TalonSRX(1);

      Hardware.Hopper.topSim = ((WPI_TalonSRX) Hardware.Hopper.top).getSimCollection();
      Hardware.Hopper.bottomSim = ((WPI_TalonSRX) Hardware.Hopper.bottom).getSimCollection();
    }

    state = State.DISABLED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(state) {
      case RUNNING :
        break;
      case DISABLED :
        stopMotors();
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  /** Runs all the motors */
  public void runMotors() {
    
  }

  /** Runs only the top motor */
  public void runTopMotor() {
  
  }

  /** Runs only the bottom motor */
  public void runBottomMotor() {

  }

  /** Stops the top motor */
  public void stopTopMotor() {
    Hardware.Hopper.top.set(ControlMode.PercentOutput, 0);
  }

  /** Stops the bottom motor */
  public void stopBottomMotor() {
    Hardware.Hopper.bottom.set(ControlMode.PercentOutput, 0);
  }

  /** Stops all the motors */
  public void stopMotors() {
    stopTopMotor();
    stopBottomMotor();
  }

  /** Sets the state of the subsystem */
  public void setState(State state) {
    this.state = state;
  }

  /** Gets the number of balls currently in the hopper */
  public int getBallNum() {
    return 0;
  }
}
