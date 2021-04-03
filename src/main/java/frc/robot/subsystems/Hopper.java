// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  public static enum State {
    RUNNING,
    DISABLED
  }

  /** Creates a new Hopper. */
  public Hopper() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run() {

  }

  public void stopMotors() {

  }

  public void setState(State state) {

  }

  public int getBallNum() {
    return 0;
  }
}
