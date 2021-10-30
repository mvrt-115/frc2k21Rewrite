
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
    public static final int PID_IDX = 0;
    public static final int TIMEOUT_MS = 10;
    
    public static final double MAX_VOLTAGE = 10.0; // Volts

    public static class Limelight {
        public static final double MOUNT_ANGLE = 15; //degrees
        public static final double HEIGHT_IN = 45; // inches  
        public static final double TARGET_HEIGHT_IN = 98.25;
    }
    
    public static class LED
    {
        public static final int kLedLength = 60;
        public static final int kColorBlockLength = 5;
        public static final double kTimeToMove = 0.1;
    
        public static final int LED_PORT = 6;

        public static final int MAX_BALLS = 5;

        public static final int[] purple = {85,5,117};
        public static final int[] gold = {255,196,16};
    }
    
    public static class Flywheel {
        public static final double P = 0.21;
        public static final double I = 0;
        public static final double D = 6.9;
        public static final double F = 0.058;
        public static final double GEAR_RATIO = 20.0 / 34.0;
        public static final double ACCEPTABLE_ERROR = 100;
        public static final int RPM_STORED = 5;
        public static final int MAX_VOLTAGE_COMPENSATION = 10;
        public static final int TICKS_PER_REVOLUTION = 2048;
        public static final double FLYWHEEL_VELOCITY_COMP_MPS2 = 0;
        public static final double FLYWHEEL_RADIUS_IN = 2;
        public static final double FLYWHEEL_ANGLE_DEG = 70;   
    }

    public static class Intake {
        public static final int PIVOT_ID = 3;
        public static final int ROLLER_ID = 5;
        public static final int FUNNEL_ID = 31;

        public static final int LIMIT_SWITCH_ID = 0;

        public static final int PIVOT_SIM_ID = 1;
        public static final int ROLLER_SIM_ID = 2;
        public static final int FUNNEL_SIM_ID = 4;

        public static final double PIVOT_STOWED_TICKS = 100;
        public static final double PIVOT_DEPLOYED_TICKS = 900;

        public static final double ROLLER_SPEED = 0.6;
        public static final double FUNNEL_SPEED = 0.7;

        public static final double PIVOT_TICKS_PER_REVOLUTION = 4096;
        public static final double PIVOT_GEAR_RATIO = 10;

        public static final double MARGIN_OF_ERROR_TICKS = 200;

        public static final double P = 0.57;
        public static final double I = 0;
        public static final double D = 0;
        public static final double FF = 0.31;
        public static final double ROTATIONAL_INERTIA = 0.1;
        public static final double PIVOT_MASS = 68.03;
        public static final double PIVOT_LENGTH = 0.1;
        public static final double PIVOT_MIN_ANGLE = -Math.PI/12;
        public static final double PIVOT_MAX_ANGLE = Math.PI/2 + Math.PI/12;
    }

    public static class Drivetrain{
        //Constants
        //zero and ask mech for vals
        public static final double kLimelightFF = 0.042;
        public static final double kLimelightP = 0.04;

        public static final double kTrackScrubFactor = 1.0469745223;
        public static final double kTrackWidthInches = 24.2; // inches
        public static final double kTrackWidthMeters = .5883; // meters
        public static final double kWheelDiameterMeters = .158; // meters      // .1450848;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // meters
                
        public static final double kMaxVelocityMetersPerSecond = 1.5; // m/s
        public static final double kMaxAccelerationMetersPerSecondSq = 2;// m/s^2
        public static final double kDriveGearRatio = (46.0/9) * (44.0/20); // ticks
        public static final int kFalconTicksPerRotation = 2048; // ticks/rotation
                
        public static final double kDriveS = 1.0;
        public static final double kDriveV = 0.8; 
        public static final double kDriveA = 0.07;   

        public static final double kDriveP = 0; 
        public static final double kDriveI = 0;
        public static final double kDriveD = 0.0;
                
        public static final double kSensitivity = 0.90;
        public static final double kWheelDeadband = 0.02;
        public static final double kThrottleDeadband = 0.02;
                
        public static final double kRobotMass = 125.0; // kg
        public static final double kRotationalInertia = 20.0;

        public static final double kAlignP = 0.0051;
        public static final double kAlignI = 0.00051;
        public static final double kAlignD = 0.0018;
        public static final double kAlignff = 0.0033;

        public static final double kAcceptableAlignError = 1.5; 

        public static final double kIntegralRange = 1;
    }

    //provides constants for the climber
    public static class Climber
    {
        //PID Constants
        public static final double kElevatorP = 1;
        public static final double kElevatorI = 0.04;
        public static final double kElevatorD = 0.4;
        public static final double kElevatorClimbOutput = 0.257;
        
        //Location Ticks
        public static final double kClimbHeight = 320000;
        public static final double kElevatorZero = 80000;

        //Servo angles
        public static final double kServoRatchet = 1;
        public static final double kServoUnRatchet = 0;

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
