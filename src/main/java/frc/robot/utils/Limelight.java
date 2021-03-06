// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  private RollingAverage tx;
  private RollingAverage ty;
  private NetworkTable limelight;

  public static enum LED_STATE {
    DEFAULT, ON, OFF, BLINKING;
  }

  public static enum CAM_MODE {
    VISION_WIDE, DRIVER, VISION_ZOOM;
  }

  /** Creates a new LimelightWrapper. */
  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    ty = new RollingAverage(20);
    tx = new RollingAverage(20);
    setLED(LED_STATE.ON);
    setPipeline(CAM_MODE.VISION_WIDE);
  }

  @Override
  public void periodic() {
    // update ty and tx
    updateEntry("ty", ty);
    updateEntry("tx", tx);
  }

  public void setLED(LED_STATE newState) {
    switch (newState) {
      case ON:
        limelight.getEntry("ledMode").setNumber(3);
        break;
      case OFF:
        limelight.getEntry("ledMode").setNumber(1);
        break;
      case BLINKING:
        limelight.getEntry("ledMode").setNumber(2);
        break;
      case DEFAULT:
        limelight.getEntry("ledMode").setNumber(0);
        break;
    }
  }

  public void setPipeline(CAM_MODE newMode) {
    switch (newMode) {
      case VISION_WIDE:
        limelight.getEntry("pipeline").setNumber(0);
        break;
      case VISION_ZOOM:
        limelight.getEntry("pipeline").setNumber(1);
        break;
      case DRIVER:
        limelight.getEntry("pipeline").setNumber(2);
        break;
    }
  }

  /**
   * Called periodically to update the rolling average values for a given entry (ty or tx)
   * @param key         The entry from limelight to change ("ty" or "tx")
   * @param rollingAvg  The rolling average to update (ty or tx)
   */
  private void updateEntry(String key, RollingAverage rollingAvg) {
    if (targetsFound())
      rollingAvg.updateValue(limelight.getEntry(key).getDouble(0.0));
  }

  /**
   * Get vertical angle
   * 
   * @return angle (degrees)
   */
  public double getTY() {
    return ty.getAverage();
  }

  /**
   * Get horizontal angle
   * 
   * @return angle (degrees)
   */
  public double getTX() {
    return tx.getAverage();
  }

  /**
   * Uses trig to calculate the distance from the limelight to the target
   * 
   * @return  the horizontal distance from the target in inches
   */
  public double getDistanceFromTarget() {
    double height = Constants.Limelight.TARGET_HEIGHT_IN - Constants.Limelight.HEIGHT_IN;
    double offsetAngle = Math.toRadians(Constants.Limelight.MOUNT_ANGLE + ty.getAverage());
    double distance = height / Math.tan(offsetAngle);

    return distance;
  }

  /**
   * Whether limelight has found any valid targets
   * 
   * @return boolean
   */
  public boolean targetsFound() {
    int tv = (int) limelight.getEntry("tv").getDouble(0);
    if (tv == 1)
      return true;
    return false;
  }
}
