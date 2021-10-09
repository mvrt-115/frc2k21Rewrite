/**
 * ClimberDownCommand.java
 * @version 1.0
 * @since 4/16/21
 * This command is used for bringing the elevator down to almost the bottom.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberDownCommand extends CommandBase {

  //The Climber subsystem that is used by this class
  private Climber climber;
  
  /**
   * @param climber climber(subsystem) that is to be used
   */
  public ClimberDownCommand(Climber climber) 
  {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  /**
   * Initializes the elevator state to zeroing so it is going down.
   */
  public void initialize() {
    climber.setElevatorState(Climber.ElevatorState.GOING_DOWN);
  }

  @Override
  /**
   * Does nothing
   */
  public void execute() {}

  @Override
  /**
   * Does nothing
   */
  public void end(boolean interrupted) {}

  @Override
  /**
   * @return boolean always returns true
   */
  public boolean isFinished() {
    return true;
  }
}
