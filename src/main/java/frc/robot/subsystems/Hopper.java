// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** 
* The hopper subsystem brings the balls from the intake
* to the flywheel. The hopper is controlled by 2 parts, 
* the top and bottom.
*/
public class Hopper extends SubsystemBase {
  /** use short names for both motors in the hopper  */
  private BaseTalon bottom, top; //becuase the short names makes code easier to read 
  //and understand whats going on.
  public DigitalInput breakbeamTop, breakbeamBot;
  
  // amount of balls currently in hopper  
  private int balls;
  
  public boolean prevBotState = true;
  public boolean prevTopState = true;

  // last time a ball passed top/bottom breakbeam
  public double lastBotTime;
  public double lastTopTime;

  double topStart = -1;
  
  /** Creates a new Hopper. */
  public Hopper() {
    bottom = new TalonFX(2);
    top = new TalonFX(7);
    
    breakbeamBot = new DigitalInput(9);
    breakbeamTop = new DigitalInput(7);

    lastTopTime = Timer.getFPGATimestamp();
    lastBotTime = Timer.getFPGATimestamp();

    balls = 0;

    top.configVoltageCompSaturation(Constants.MAX_VOLTAGE);
    bottom.configVoltageCompSaturation(Constants.MAX_VOLTAGE);

    bottom.enableVoltageCompensation(true);
    top.enableVoltageCompensation(true);
  }

  /**
   * Resets the hopper (sets the amount of balls in the hopper to 0)
   */
  public void resetBalls() {
    balls = 0;
  }

  
  @Override
  public void periodic() {  
    
    countBalls();

    adjustTopBelt();
    
    log();
  }

  /**
   * If a ball has just entered the hopper, run the top hopper a bit to make it go up and not jam the hopper
   */
  public void adjustTopBelt() {
    if(topStart != -1 && Timer.getFPGATimestamp() - topStart > 1) {
      stopTopMotor();
      topStart = -1;
    } else if(topStart != -1) {
      runTopMotor(0.2);
    }
  }

  /**
   * checks if a ball has entered or left the hopper and counts accordingly
   */
  public void countBalls() {
    //if the breakbeamBot goes from unbroken to broken, then increase the amount of balls
    if(!breakbeamBot.get() && prevBotState){
      if(Timer.getFPGATimestamp() - lastBotTime > .3){
        balls++;
        lastBotTime = Timer.getFPGATimestamp();
      }
    }
    
    //if the breakbeamTop goes from broken to unbroken, then decrease the amount of balls
    if(!breakbeamTop.get() && prevTopState){
      if(Timer.getFPGATimestamp() - lastTopTime > .3){
        lastTopTime = Timer.getFPGATimestamp();
        balls--;
     }
    }      
    
    // makes sure that the hopper doesnt miscount balls, so the amount of balls is always between 0 and 5
    balls = Math.min(Math.max(0, balls), 5);

    prevBotState = breakbeamBot.get();
    prevTopState = breakbeamTop.get();
  }
  

  /**
   * Runs the hopper
   * 
   * if less than 3 balls in hopper (the last ball will just sit on the ground)
   *    run the bottom motor if the intake is intaking balls
   * 
   *    if there already is a ball at the top of the bottom section
   *        run the top motor until the top break beam is hit
   *    otherwise
   *        stop the top motor
   * otherwise
   *    stop all motors
   * 
   * @param intaking  whether or not the intake is running
   */
  public void runHopper(boolean intaking) {
    if(balls <= 3) {
      if(intaking)
        runBottomMotor(0.65);

      // gets the hopper to adjust the top belt to prevent jamming
      if(!breakbeamBot.get() && (balls == 0 || balls == 1)) {
        topStart = Timer.getFPGATimestamp();
      }
       if(balls == 2 && breakbeamTop.get()) // if break beam is not broken
         runTopMotor(0.25);
       else
        stopTopMotor();
      
    } else {
      stop();
    }
  }

  
  /** Runs only the top motor. (Should be used to de-jam only or manual) */
  public void runTopMotor(double speed) {
    top.set(ControlMode.PercentOutput, speed);
  }
  
  /** Runs only the bottom motor. (Should be used to de-jam only or manual) */
  public void runBottomMotor(double speed) {
    bottom.set(ControlMode.PercentOutput, speed);
  }
  
  /** Stops only the top motor */
  public void stopTopMotor() {
    top.set(ControlMode.PercentOutput, 0);
  }
  
  /** Stops only the bottom motor */
  public void stopBotMotor() {
    top.set(ControlMode.PercentOutput, 0);
  }
  
  /** Stops both motors in the hopper */
  public void stop() {
    top.set(ControlMode.PercentOutput, 0);
    bottom.set(ControlMode.PercentOutput, 0);
  }
  
  public void log() {
    SmartDashboard.putBoolean("Bottom breakbeam", breakbeamBot.get());
    SmartDashboard.putBoolean("Top breakbeam", breakbeamTop.get());    
    SmartDashboard.putNumber("Balls", balls);
  }
  
  /**
   * Gets the number of balls in the hopper
   * @return Returns the balls in the hopper
   */
  public int getBallsInHopper() {
    return balls;
  }

  public void setBallsInHopper(int newBalls) {
    balls = newBalls;
  }
}