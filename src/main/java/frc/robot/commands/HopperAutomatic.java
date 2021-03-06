// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

/**
 * HopperAutomatic runs the motors until 
 */
public class HopperAutomatic extends CommandBase {
  private Hopper hopper;
  private Intake intake;

  /** Creates a new HopperAutomatic. */
  public HopperAutomatic(Hopper hopper, Intake intake) {
    this.hopper = hopper;
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.runHopper(intake.getState() != IntakeState.DISABLED);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}