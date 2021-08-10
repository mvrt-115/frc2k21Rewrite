package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Flywheel.FlywheelState;

public class SetFlywheelRPM extends CommandBase {
  /** Creates a new SetFlywheelRPM. */
  private Flywheel flywheel;
  private double desiredRPM;
  public SetFlywheelRPM(Flywheel flywheel, double desiredRPM) {
    this.flywheel = flywheel;
    this.desiredRPM = desiredRPM;

    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setTargetRPM(desiredRPM);
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
    return flywheel.getFlywheelState() == FlywheelState.ATSPEED;
  }
}