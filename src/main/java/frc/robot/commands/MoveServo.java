// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ElevatorState;

public class MoveServo extends CommandBase {
  /** Creates a new MoveSirvo. */
  Climber climber;
  boolean move;
  public MoveServo(Climber climber, boolean move) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.move = move;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(move)
      climber.setElevatorState(Climber.ElevatorState.SERVO_TEST_UNRATCHET);
    else
      climber.setElevatorState(ElevatorState.SERVO_TEST_RATCHET);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
