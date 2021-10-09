// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    flywheel.setTargetRPM(flywheel.getRequiredRPM());
    // flywheel.setTargetRPM(1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(hopper.getBallsInHopper() == 0 && topStart == -1) {
    //     topStart = Timer.getFPGATimestamp();
    //   }
    if(hopper.getBallsInHopper() == 0 && startDriving == -1) {
      // topStart = -1;
      flywheel.setTargetRPM(0);
      hopper.stop();
      if(startDriving == -1)
        startDriving = Timer.getFPGATimestamp();
    }
    if(hopper.getBallsInHopper() != 0 ) {
      if(flywheel.getFlywheelState() == FlywheelState.ATSPEED) {
        hopper.runTopMotor(0.4);
        hopper.runBottomMotor(0.7);
      } else {
        hopper.runTopMotor(0);
        hopper.runBottomMotor(0);
      }
  
      
      SmartDashboard.putNumber("shooting auton", 123);
    } 
    if(startDriving != -1) {
      drivetrain.setDrivetrainMotorSpeed(-0.5, -0.5);
      SmartDashboard.putNumber("shooting auton", 143252);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startDriving > 2 && startDriving != -1;
  }
}
