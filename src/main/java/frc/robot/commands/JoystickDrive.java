package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * A command that runs throughout the teleoperated period, and constantly
 * checks the status of the joysticks to determine what output needs be
 * done by the motors, and sets the motors as such.
 */
public class JoystickDrive extends CommandBase {
  
  /**
   * Makes sure that the Drivetrain.java subsystem object exists for use in the
   * command
   */
  public JoystickDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Sends status of buttons and triggers on the trigger to the cheesyIshDrive
   * method in Drivetrain.java to have a certain drivetrain motor output.
   */
  @Override
  public void execute() 
  {
    RobotContainer.drivetrain.cheesyIshDrive(RobotContainer.getThrottle(), RobotContainer.getWheel(), RobotContainer.getQuickTurn());

    SmartDashboard.putNumber("Throttle", RobotContainer.getThrottle());
    SmartDashboard.putNumber("Wheel", RobotContainer.getWheel());
    SmartDashboard.putBoolean("QuickTurn", RobotContainer.getQuickTurn());
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
    RobotContainer.drivetrain.stopDrivetrainMotors();
  }

  // Returns true when the command should end (which is not until the robot command is interrupted)
  @Override
  public boolean isFinished() {
    return false;
  }
}