// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class ExampleSubsystem {
        public static final double SPEED = 0.5;
    }

    public static class Climber
    {
        //PID Constants
        //ToDo:zero constants out and manually do it
        public static final double kElevatorP = 0.5;
        public static final double kElevatorI = 0.2;
        public static final double kElevatorD = 0.2;
        public static final double kElevatorHoldOutput = -.2;
        public static final double kElevatorClimbOutput = -.58;
        
        //Location Ticks
        public static final double kClimbHeight = 370_000;
        public static final double kElevatorZero = 1_000;
        public static final int kClimbTicks = 80_000;

        public static final double kServoRatchet = 0;
        public static final double kServoUnRatchet = 0.4;

        //TODO: FIND REAL VALUES FOR THESE NUMBERS
        public static final double CARRIAGE_MASS = convertPoundsToKg(15);
        public static final double GEAR_REDUCTION = 10.0;
        public static final double PULLEY_RADIUS = Units.inchesToMeters(2);

        public static final double TICKS_PER_ROTATION = 4096;
        public static final double DISTANCE_PER_PULSE = 2.0 * Math.PI * PULLEY_RADIUS / GEAR_REDUCTION / TICKS_PER_ROTATION;
        public static final double MIN_HEIGHT = 1_000.0 * DISTANCE_PER_PULSE, MAX_HEIGHT = 370_000.0 * DISTANCE_PER_PULSE;
        public static final int CHANNEL_A = 0, CHANNEL_B = 1;
        
        private static double convertPoundsToKg(double pounds)
        {
            return pounds * 0.45359237; 
        }
    }
    public static final int kPIDIdx = 0;
    public static final int kTimeoutMs = 10;
    public static final boolean kCompBot = true;
}
