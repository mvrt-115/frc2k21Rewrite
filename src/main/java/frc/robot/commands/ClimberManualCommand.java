// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class ClimberManualCommand extends CommandBase {
  private Climber climber;

  public ClimberManualCommand(Climber climber)
  {
    this.climber = climber;
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("in");
    this.climber.setElevatorState(Climber.ElevatorState.MANUAL_OVERRIDE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    this.climber.setElevatorState(Climber.ElevatorState.HOLD);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.getContainer().getLeft() == 0;
  }
}
