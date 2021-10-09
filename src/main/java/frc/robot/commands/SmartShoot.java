/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Flywheel.FlywheelState;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.CAM_MODE;
import frc.robot.utils.Limelight.LED_STATE;

public class SmartShoot extends CommandBase {
  private Flywheel flywheel;
  private Hopper hopper;
  boolean stop = false;
  double topStart = -1;
  Limelight limelight;
  boolean auton;

  /**
   * Creates a new SmartShoot.
   */
  public SmartShoot(Flywheel flywheel, Hopper hopper, Limelight limelight, boolean auton) {

    this.flywheel = flywheel;
    this.hopper = hopper;
    auton = this.auton;

    this.limelight = limelight;
    this.auton = auton;

    addRequirements(flywheel, hopper);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLED(LED_STATE.ON);
    limelight.setPipeline(CAM_MODE.VISION_WIDE);
    flywheel.setTargetRPM(flywheel.getRequiredRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    if(flywheel.getFlywheelState() == FlywheelState.ATSPEED) {
        hopper.runTopMotor(0.6);
        hopper.runBottomMotor(0.8);
      } else {
        hopper.runTopMotor(0);
        hopper.runBottomMotor(0);
      }
  
      if(hopper.getBallsInHopper() == 0 && topStart == -1) {
        topStart = Timer.getFPGATimestamp();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    flywheel.setTargetRPM(0);
    // limelight.setLED(LED_STATE.OFF);
    // limelight.setPipeline(CAM_MODE.DRIVER);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return auton && Timer.getFPGATimestamp() - topStart > 0.3 && topStart != -1;
  }
}
