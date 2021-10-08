/**
 * ClimberUpCommand.java
 * @version 1.0
 * @since 4/16/21
 * This command is used for moving the elevator up.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberUpCommand extends CommandBase {
  
  //The Climber subsystem that is used by this class
  private Climber climber;

  /**
   * @param climber the climber(sybsystem) used by this class
   */
  public ClimberUpCommand(Climber climber) 
  {
    this.climber = climber;
    addRequirements(this.climber);
  }

  @Override
  /**
   * Sets the elevator state to climbing
   */
  public void initialize() {
      climber.setElevatorState(Climber.ElevatorState.GOING_UP);
  }

  @Override
  /**
   * Does nothing
   */
  public void execute() {

      climber.setElevatorState(Climber.ElevatorState.GOING_UP);
  }

  @Override
  /**
   * Does nothing
   */
  public void end(boolean interrupted) {}

  @Override
  /**
   * Always returns true
   */
  public boolean isFinished() {
    return true;
  }
}
