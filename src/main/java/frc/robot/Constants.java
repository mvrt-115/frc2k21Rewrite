/**
 * Constants.java
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 * @version 1.0
 * @since 4/16/2021
 */

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants 
{
    //provides constants for the climber
    public static class Climber
    {
        //PID Constants
        public static final double kElevatorP = 1;
        public static final double kElevatorI = 0.04;
        public static final double kElevatorD = 0.4;
        public static final double kElevatorClimbOutput = 0.257;
        
        //Location Ticks
        public static final double kClimbHeight = 37_000;
        public static final double kElevatorZero = 1_000;

        //Servo angles
        public static final double kServoRatchet = 0;
        public static final double kServoUnRatchet = 1;

        //Other usefull constants
        public static final double TICKS_PER_ROTATION = 4096;
        public static final double ACCEPTABLE_AMOUNT = 0.2;
        
        //Simulating a Climber
        public static final double CARRIAGE_MASS = convertPoundsToKg(15);
        public static final double GEAR_REDUCTION = 10.0;
        public static final double PULLEY_RADIUS = Units.inchesToMeters(2);
        public static final double DISTANCE_PER_PULSE = 2.0 * Math.PI * PULLEY_RADIUS / GEAR_REDUCTION / TICKS_PER_ROTATION;
        public static final double MIN_HEIGHT = 0.0;
        public static final double MAX_HEIGHT = kClimbHeight * DISTANCE_PER_PULSE;
        public static final int CHANNEL_A = 0, CHANNEL_B = 1;
        
        /**
         * @param pounds
         * @return kilograms
         */
        private static double convertPoundsToKg(double pounds)
        {
            return pounds * 0.45359237; 
        }
    }
    
    public static final int kPIDIdx = 0;            //the pid index
    public static final int kTimeoutMs = 10;        //the time out ms
    public static final boolean kCompBot = true;    //whether or not the robot is a competion bot
}
