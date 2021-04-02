// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
* constants. This class should not be used for any other purpose. All constants should be declared
* globally (i.e. public static). Do not put anything functional in this class.
*
* <p>It is advised to statically import this class (or one of its inner classes) wherever the
* constants are needed, to reduce verbosity.
*/
public final class Constants {
  /** Hopper subsystem constants*/
  public static class Hopper {
    public static final double KP = 0.1;
    public static final double KI = 0.01;
    public static final double KD = 0.1;
    
    public static final int MANUAL_RPM = 1000;
    
    public static final int AUTOMATIC_BASE_RPM = 1000;
    public static final int AUTOMATIC_SLOPE = 1;
    
    // public static final int AUTOMATIC_ONE_BALL = 1000;
    // public static final int AUTOMATIC_TWO_BALL = 2000;
    // public static final int AUTOMATIC_THREE_BALL = 3000;
    // public static final int AUTOMATIC_FOUR_BALL = 4000;
  }
}
