// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class RunIntake extends CommandBase {

  private boolean startStop;
  private Intake intake;
  private Hopper hopper;

  /**
   * Command that runs the intake
   * @param intake    The intake subsystem
   * @param startStop Whether to start or stop the command
   */
  public RunIntake( Intake intake, Hopper hopper,  boolean startStop ) {

    this.startStop = startStop;
    this.intake = intake;
    this.hopper = hopper;

    addRequirements( intake, hopper );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(startStop) {
      if(intake.getState() != IntakeState.INTAKING)
        intake.setState(IntakeState.DEPLOYING);
      else
        hopper.runHopper();
    }
    else {
      intake.setState(IntakeState.STOWING);
      hopper.stop();
    }
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