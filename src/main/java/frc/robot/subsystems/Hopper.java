// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;

/** 
 * The hopper subsystem brings the balls from the intake
 * to the flywheel. The hopper is controlled by 2 parts, 
 * the top and bottom.
 * 
 * Strategy (Hopper is empty):
 * 
 * 1st ball) Run bottom till the middle break beam returns true (the ball should stay at the top 
 *    of the bottom section since the top motor isn't running) - if ball is hopping up because the
 *    bottom motor is running too fast add a boolean to "remember" mid was hit
 * 2st ball) Run the bottom till the bottom break beam is hit; run the bottom and top till the top 
 *     break beam is hit
 * 3rd/4th ball) Repeat steps for the first and second ball except for moving the bottom balls to the top
 *    since the top is already full
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

    Hardware.Hopper.diTop = new DigitalInput(2);
    Hardware.Hopper.diMid = new DigitalInput(3);
    Hardware.Hopper.diBot = new DigitalInput(4);

    state = State.DISABLED;
  }

  /** Handles state */
  @Override
  public void periodic() {
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

  /** Runs all motors as necessary until the hopper is completely full */
  public void runTillFull() {
    if(botIsFull() && !topIsFull()) {
      runTopTillFull();
      runBottomMotor();
    } else if(!botIsFull()) {
      runBottomTillFull();
    }
  }

  /** Runs the top motor till full */
  public void runTopTillFull() {
    if(Hardware.Hopper.diTop.get()) {
      state = State.DISABLED;
    } else {
      state = State.RUNNING;

      Hardware.Hopper.top.set(ControlMode.PercentOutput, Constants.Hopper.OUT);
    }
  }

  /** Runs the bottom motor till full */
  public void runBottomTillFull() {
    if(!(Hardware.Hopper.diMid.get() && Hardware.Hopper.diBot.get())) {
      state = State.RUNNING;
      
      Hardware.Hopper.bottom.set(ControlMode.PercentOutput, Constants.Hopper.OUT);
    } else {
      state = State.DISABLED;
    }
  }

  /** Runs all the motors */
  public void runMotors() {
    runTopMotor();
    runBottomMotor();
  }

  /** Runs only the top motor. (Should be used to de-jam only or manual) */
  public void runTopMotor() {
    Hardware.Hopper.top.set(ControlMode.PercentOutput, Constants.Hopper.OUT);
  }

  /** Runs only the bottom motor (Should be used to de-jam only or manual) */
  public void runBottomMotor() {
    Hardware.Hopper.bottom.set(ControlMode.PercentOutput, Constants.Hopper.OUT);
  }

  /** Stops all the motors */
  public void stopMotors() {
    stopTopMotor();
    stopBottomMotor();
  }

  /** Stops the top motor */
  public void stopTopMotor() {
    Hardware.Hopper.top.set(ControlMode.PercentOutput, 0);
  }

  /** Stops the bottom motor */
  public void stopBottomMotor() {
    Hardware.Hopper.bottom.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Returns if the top section is full
   * @return returns true or false for if the top is full
   */
  public boolean topIsFull() {
    return Hardware.Hopper.diTop.get();
  }

  /**
 * Returns if the bottom section is full
 * @return returns true or false for if the bottom is full
 */
  public boolean botIsFull() {
    return Hardware.Hopper.diMid.get() && Hardware.Hopper.diBot.get();
  }

  /** Sets the state of the subsystem */
  public void setState(State state) {
    this.state = state;
  }

  // uncomment and use if need to vary speed based on ball num

  // /** Gets the number of balls currently in the hopper */
  // public int getBallNum() {
  //   int n = 0;

  //   if(Hardware.Hopper.diTop.get())
  //     n += 2;
    
  //   if(Hardware.Hopper.diMid.get())
  //     n++;

  //   if(Hardware.Hopper.diBot.get())
  //     n++;
      
  //   return n;
  // }
}
