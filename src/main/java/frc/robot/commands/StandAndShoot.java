// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Flywheel.FlywheelState;

public class StandAndShoot extends CommandBase {
  Hopper hopper; 
  Flywheel flywheel; 
  Drivetrain drivetrain;
  double topStart = -1;
  double startDriving = -1;
  
  /** Creates a new StandAndShoot. */
  public StandAndShoot(Hopper hopper, Flywheel flywheel, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;
    this.flywheel = flywheel;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.setBallsInHopper(3);
    topStart = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(hopper.getBallsInHopper() != 0 && !(Timer.getFPGATimestamp() - topStart > 0.3 && topStart != -1)) {
      if(flywheel.getFlywheelState() == FlywheelState.ATSPEED) {
        hopper.runTopMotor(0.4);
        hopper.runBottomMotor(0.7);
      } else {
        hopper.runTopMotor(0);
        hopper.runBottomMotor(0);
      }
  
      if(hopper.getBallsInHopper() == 0 && topStart == -1) {
        topStart = Timer.getFPGATimestamp();
      }
    } else {
      flywheel.setTargetRPM(0);
      hopper.stop();
      if(startDriving == -1)
        startDriving = Timer.getFPGATimestamp();

      drivetrain.setDrivetrainMotorSpeed(0.3, 0.3);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setDrivetrainMotorSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startDriving > 10 && startDriving != -1;
  }
}
