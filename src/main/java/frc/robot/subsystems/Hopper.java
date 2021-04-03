// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;

public class Hopper extends SubsystemBase {

  TalonSRXSimCollection topSimCollection;
  TalonSRXSimCollection botSimCollection;

  /** use short names for both motors in the hopper  */
  BaseTalon bottom, top; //becuase the short names makes code easier to read 
                        //and understand whats going on.
  DigitalInput breakbeamTop, breakbeamBot, breakbeamMid;

  // amount of balls currently in hopper  
  private int ballsInHopper;

  /** if there are max num balls in hopper */
  private boolean ballsMaxed;

  public boolean prevBreakbeamBotState;
  public boolean prevBreakbeamTopState;
  public boolean prevBreakbeamMidState;
  
  /** Creates a new Hopper. */
  public Hopper() {
    if (Robot.isReal()) {
      Hardware.Hopper.bottom = new TalonFX(10);
      Hardware.Hopper.top = new TalonFX(11);
      
    } else {
      Hardware.Hopper.bottom = new WPI_TalonSRX(10);
      Hardware.Hopper.top = new WPI_TalonSRX(11);

      botSimCollection = ((WPI_TalonSRX)Hardware.Hopper.bottom).getSimCollection();
      topSimCollection = ((WPI_TalonSRX)Hardware.Hopper.top).getSimCollection();
    }
    bottom = Hardware.Hopper.bottom;
    top = Hardware.Hopper.top;

    Hardware.Hopper.breakbeamBot = new DigitalInput(12); //arbritrarty channel
    Hardware.Hopper.breakbeamTop = new DigitalInput(12); //arbritrarty channel
    Hardware.Hopper.breakbeamMid = new DigitalInput(12); //arbritrarty channel

    breakbeamBot = Hardware.Hopper.breakbeamBot;
    breakbeamTop = Hardware.Hopper.breakbeamTop;
    breakbeamMid = Hardware.Hopper.breakbeamMid;


  }
/**
 * sets the speed of the motor located at the top front of the robot
 * @param speed - desired speed of motor [-1, 1]
 */
  public void setTopSpeed(double speed) {
    top.set(ControlMode.PercentOutput, speed);
  }

  /**
   * sets the speed of the motor located at bottom back of the robot
   * @param speed - desired speed of motor [-1, 1]
   */
  public void setBotSpeed(double speed) {
    bottom.set(ControlMode.PercentOutput, speed);
  }

  /**
   * stops both motors in the hopper
   */
  public void stop() {
    top.set(ControlMode.PercentOutput, 0.0);
    bottom.set(ControlMode.PercentOutput, 0.0);
  }



  @Override
  public void periodic() {
    //if there is ball at bottom, move until ball triggers middle breakbeam
    if (breakbeamBot.get() == false) {

      //move up until mid breakbeam is triggered
      if(!(breakbeamMid.get() == false && prevBreakbeamMidState == true)){
        setTopSpeed(0.2);
        setBotSpeed(-0.2);
      }
      else {
        stop();
      }
      
    }

    //if top breakbeam is triggered stop moving hopper and 
    //do not allow movement unless called shoot command is called
    if(breakbeamTop.get() == false){
      ballsMaxed = true; 
    }

    //if the breakbeamBot goes from unbroken to broken, then increase the amount of balls
    if(breakbeamBot.get() == false && prevBreakbeamBotState == true){
      ballsInHopper ++;
    }

    //if the breakbeamTop goes from broken to unbroken, then decrease the amount of balls
    if(breakbeamTop.get() == true && prevBreakbeamBotState == false){
      ballsInHopper --;
    }
    
    prevBreakbeamBotState = breakbeamBot.get();
    prevBreakbeamMidState = breakbeamMid.get();
    prevBreakbeamTopState = breakbeamTop.get();

    log();
  }

  public void log() {
    SmartDashboard.putBoolean("Bot breakbeam", breakbeamBot.get());
    SmartDashboard.putBoolean("Mid breakbeam", breakbeamMid.get());
    SmartDashboard.putBoolean("Top breakbeam", breakbeamTop.get());

    SmartDashboard.putNumber("Balls in Hopper", ballsInHopper);
    SmartDashboard.putBoolean("Max balls reached", ballsMaxed);

  }
}
