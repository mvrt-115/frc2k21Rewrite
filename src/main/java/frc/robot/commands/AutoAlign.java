  
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.CAM_MODE;
import frc.robot.utils.Limelight.LED_STATE;

/**
 * A command that aligns the robot to the target using the limelight system
 * to calculate the angle needed to turn, and then uses Drivetrain methods
 * to turn the robot until the robot is in line with the target.
 */
public class AutoAlign extends CommandBase {

  Drivetrain  drivetrain;
  Limelight limelight;

  /**
   * Makes sure that the drivetrain subsystem object exists for use in the
   * command
   */
  public AutoAlign(Drivetrain drivetrain, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  /**
   * Resets the gyro to prepare for turn calculations
   */
  @Override
  public void initialize() 
  {
    drivetrain.resetGyro();
    limelight.setLED(LED_STATE.ON);
    limelight.setPipeline(CAM_MODE.VISION_WIDE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Gets the angular error to the target, and then uses the given information
   * to turn the motors at a certain percent output.
   */
  @Override
  public void execute() 
  {
    drivetrain.alignToTarget();
  }

  // Called once the command ends or is interrupted.
  /**
   * When the program ends or is interrupted, the method stops the drivetrain motors
   * from moving.
   * @param interrupted     whether the command has been interrupted or not
   */
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.stopDrivetrainMotors();
    limelight.setLED(LED_STATE.OFF);
    limelight.setPipeline(CAM_MODE.DRIVER);
  }

  // Returns true when the command should end.
  /**
   * Checks for whether the alignment angle has been reached, and if so, it returns true,
   * triggering the end sequence.
   * @return true if the robot has turned the through the angular error to be aligned,
   *          false otherwise 
   */
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getHorizontalAngleError()) < 3;
  }
}