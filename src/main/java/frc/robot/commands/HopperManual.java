// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * HopperManual runs the hopper indefinetely for as long as the button is held down
 */
public class HopperManual extends CommandBase {
  private Hopper hopper;
  private boolean run;

  /** Creates a new HopperManual. */
  public HopperManual(Hopper hopper, boolean run) {
    this.hopper = hopper;
    this.run = run;

    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(run) {
      hopper.setState(Hopper.State.DISABLED);
    } else { 
      hopper.setRPM(Constants.Hopper.MANUAL_RPM);
      hopper.setState(Hopper.State.RUNNING);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
