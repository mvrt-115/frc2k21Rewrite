/**
 * ClimberTestServoRatchet.java
 * @version 1.0
 * @since 4/16/21
 * This command is used for testing the elevator servo.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ElevatorState;

public class ClimberTestServoRatchet extends CommandBase {

  private Climber climber;
  
  /** Creates a new ClimberTestServo. */
  public ClimberTestServoRatchet(Climber climber) 
  {
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setElevatorState(ElevatorState.SERVO_TEST_RATCHET);
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
