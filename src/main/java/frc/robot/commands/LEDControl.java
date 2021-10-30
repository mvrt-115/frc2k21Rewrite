// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LED; 

public class LEDControl extends CommandBase {
  /** Creates a new LEDControl. */
  private LED led;
  private int lastBallCount;
  private final int blockColorLength;
  private Supplier<Boolean> getElevatorUp, getAutoAlign;
  private Supplier<Integer> getBallsInHopper;
  private Supplier<Double> getHorizontalAngleError;
  private boolean endgame;

  public LEDControl(LED ledIn, Supplier<Integer> getBallsInHopperIn, Supplier<Boolean> getElevatorUpIn, 
    Supplier<Boolean> getAutoAlignIn, Supplier<Double> getHorizontalAngleErrorIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    led = ledIn;
    lastBallCount = 0;
    getElevatorUp = getElevatorUpIn;
    getBallsInHopper = getBallsInHopperIn;
    getAutoAlign = getAutoAlignIn;
    getHorizontalAngleError = getHorizontalAngleErrorIn;
    blockColorLength = led.getLength()/Constants.LED.MAX_BALLS;
    endgame = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setOneColor(Color.kBlack, 0, led.getLength());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int currBallCount = getBallsInHopper.get();

    //once the elevator button is pressed and the climb starts, the LEDs get set to rainbow and move
    if(getElevatorUp.get()) {
      endgame = true;
      led.setRainbow();
    }

    if(endgame) {
      led.moveLEDs(Constants.LED.kTimeToMove);
    }
    //when AutoAlign command is being run, the leds remain yellow moving while not within error range, but turn
    // moving green when within error range
    else if(getAutoAlign.get()) {
      if(Math.abs(getHorizontalAngleError.get()) < 4) {
        led.setTwoColors(Color.kGreen, Color.kGreenYellow, blockColorLength, 0, led.getLength());
      }
      else
      {
        led.setTwoColors(Color.kYellow, Color.kLightGoldenrodYellow, blockColorLength, 0, led.getLength());
      }

      led.moveLEDs(Constants.LED.kTimeToMove);
    }
    //during teleop, the LED fills based on how many balls are in the hopper in MVRT colors, and if the hopper is full,
    // the LEDs start to move
    else {
      if(currBallCount != lastBallCount) //updates LEDs only if there is a change in number of balls
      {
        led.setTwoColors(new Color(Constants.LED.purple[0], Constants.LED.purple[1], Constants.LED.purple[2]), 
        new Color(Constants.LED.gold[0], Constants.LED.gold[1], Constants.LED.gold[2]), blockColorLength, 0, blockColorLength*currBallCount);
        led.setOneColor(Color.kBlack, blockColorLength*currBallCount, led.getLength());
      }
      if(currBallCount == Constants.LED.MAX_BALLS) //moves the LEDs if there are max number of balls
      {
        led.moveLEDs(Constants.LED.kTimeToMove);
      }
    }

    lastBallCount = currBallCount;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.stopLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
