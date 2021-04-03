// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class RunIntake extends CommandBase {

  boolean run;
  Intake intake;

  public RunIntake( Intake intake,  boolean run ) {

    this.run = run;
    this.intake = intake;

    if( run ) 
      intake.setState( IntakeState.DEPLOYING );
    else
      intake.setState( 
        ( intake.getState() == IntakeState.STOWED )
        ? IntakeState.STOWED 
        : IntakeState.STOWING
      );

    addRequirements( intake );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeState state = intake.getState();
    if( state == IntakeState.DISABLED ) return;

    if( !run && state != IntakeState.STOWED )
      intake.setState( IntakeState.STOWING );

    switch( state )
    {
      case INTAKING:
        intake.intake();
        break;

      case STOWING:
        intake.stowing();
        break;

      case DEPLOYING:
        intake.deploying();
        break;

      default: 
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setState( IntakeState.DISABLED );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.disabled;
  }
}
