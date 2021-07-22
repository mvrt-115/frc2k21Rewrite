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
    breakbeamTop = new DigitalInput(8);

    lastTopTime = Timer.getFPGATimestamp();
    lastBotTime = Timer.getFPGATimestamp();

    balls = 0;

    top.configVoltageCompSaturation(Constants.kVoltageCompensation);
    bottom.configVoltageCompSaturation(Constants.kVoltageCompensation);

    bottom.enableVoltageCompensation(true);
    top.enableVoltageCompensation(true);
  }

  
  @Override
  public void periodic() {
   
    //if the breakbeamBot goes from unbroken to broken, then increase the amount of balls
    if(!breakbeamBot.get() && prevBotState){
      if(Timer.getFPGATimestamp() - lastBotTime > .3){
        balls++;
        lastBotTime = Timer.getFPGATimestamp();
      }
    }
    
    //if the breakbeamTop goes from broken to unbroken, then decrease the amount of balls
    if(breakbeamTop.get() && !prevTopState){
      if(Timer.getFPGATimestamp() - lastTopTime > .3){
        lastTopTime = Timer.getFPGATimestamp();
        balls--;
     }
    }      
    
    prevBotState = breakbeamBot.get();
    prevTopState = breakbeamTop.get();

    if(topStart != -1 && Timer.getFPGATimestamp() - topStart > 0.2) {
      stopTopMotor();
      topStart = -1;
    }
    
    log();
  }
  

  /**
   * Runs the hopper
   * 
   * if less than 3 balls in hopper (the last ball will just sit on the ground)
   *    run the bottom motor
   * 
   *    if there already is a ball at the top of the bottom section
   *        run the top motor until the top break beam is hit
   *    otherwise
   *        stop the top motor
   * otherwise
   *    stop all motors
   */
  public void runHopper() {
    // if(ballsInHopper < 3) {
    //   runBottomMotor(0.65);

    //   if(!prevBreakbeamBotState && breakbeamBot.get()) {
    //     runTopMotor(0.65);
    //   }
    //   else if(ballsInHopper == 2 && breakbeamTop.get()) // if break beam is not broken
    //     runTopMotor(0.65);
    //   else
    //     stopTopMotor();
      
    // } else {
    //   stop();
    // }
    
    
    if(balls < 3)
      runBottomMotor(0.8);
    else
      stop();

    // if(balls == 1) {
    //   runBottomMotor(0.87);
    // }

    if(balls == 2 || balls == 3) {
      if(breakbeamTop.get())
        runTopMotor(0.3);
      else
        stopTopMotor();
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
    SmartDashboard.putBoolean("Bot breakbeam", breakbeamBot.get());
    SmartDashboard.putBoolean("Top breakbeam", breakbeamTop.get());
    
    SmartDashboard.putNumber("Balls in Hopper", balls);
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