// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class HopperAutomatic extends CommandBase {
  private Hopper hopper;
  private int lastBallNum;      

  /** Creates a new HopperAutomatic. */
  public HopperAutomatic(Hopper hopper) {
    this.hopper = hopper;
    lastBallNum = hopper.getBallNum();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.setRPM(Constants.Hopper.AUTOMATIC_BASE_RPM + (Constants.Hopper.AUTOMATIC_SLOPE * hopper.getBallNum()));
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(lastBallNum != hopper.getBallNum()) {
      hopper.setRPM(Constants.Hopper.AUTOMATIC_BASE_RPM + (Constants.Hopper.AUTOMATIC_SLOPE * hopper.getBallNum()));
      lastBallNum = hopper.getBallNum();
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setRPM(0);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hopper.getBallNum() == 4;
  }
}
