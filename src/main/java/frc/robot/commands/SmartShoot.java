/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Flywheel.FlywheelState;

public class SmartShoot extends CommandBase {
  private Flywheel flywheel;
  private Hopper hopper;

  /**
   * Creates a new SmartShoot.
   */
  public SmartShoot(Flywheel flywheel, Hopper hopper) {

    this.flywheel = flywheel;
    this.hopper = hopper;

    addRequirements(flywheel, hopper);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setTargetRPM(flywheel.getRequiredRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
      if(flywheel.getFlywheelState() == FlywheelState.ATSPEED) {
        hopper.runTopMotor(0.4);
        hopper.runBottomMotor(0.8);
      } else {
        hopper.runTopMotor(0);
        hopper.runBottomMotor(0);
      }
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flywheel.getFlywheelState() == FlywheelState.ATSPEED;
  }
}
