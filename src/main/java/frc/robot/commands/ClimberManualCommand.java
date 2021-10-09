/**
 * ClimberManualCommand.java
 * @version 1.0
 * @since 4/16/21
 * This command is used for manually controlling the elevator with the joystick.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

public class ClimberManualCommand extends CommandBase {
  
  //The Climber subsystem that is used by this class
  private Climber climber;

  /**
   * @param climber the elevator(subsystem)
   */
  public ClimberManualCommand(Climber climber)
  {
    this.climber = climber;
    addRequirements(this.climber);
  }

  @Override
  /**
   * Sets the elevator state to manual override
   */
  public void initialize() {
    this.climber.setElevatorState(Climber.ElevatorState.MANUAL_OVERRIDE);
  }

  @Override
  /**
   * Does nothing
   */
  public void execute() {}

  @Override
  /**
   * Sets the elevator state to hold
   */
  public void end(boolean interrupted) 
  {
    this.climber.setElevatorState(Climber.ElevatorState.HOLD);
  }

  @Override
  /**
   * Returns true when the joystick input is 0
   */
  public boolean isFinished() {
    // return Robot.getContainer().getLeft() == 0;
    return true;
  }
}
