// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class RunIntake extends CommandBase {

  private JoystickButton button;
  private Intake intake;

  /**
   * Command that runs the intake
   * @param intake  The intake subsystem
   * @param button  The button controlling the command (when pressed run intake, when released stop)
   */
  public RunIntake( Intake intake,  JoystickButton button ) {

    this.button = button;
    this.intake = intake;
    intake.setState( IntakeState.DEPLOYING );

    addRequirements( intake );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setState(IntakeState.STOWING);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !button.get();
  }
}
