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
    public static final int PID_IDX = 0;
    public static final int TIMEOUT_MS = 10;
    
    public static final double MAX_VOLTAGE = 10.0; // Volts

    public static class Limelight {
        public static final double MOUNT_ANGLE = 15; //degrees
        public static final double HEIGHT_IN = 45; // inches  
        public static final double TARGET_HEIGHT_IN = 98.25;
    }
    public static class Flywheel {
        public static final double P = 0.17;
        public static final double I = 0;
        public static final double D = 7;
        public static final double F = 0.058;
        public static final double GEAR_RATIO = 20.0 / 34.0;
        public static final double ACCEPTABLE_ERROR = 100;
        public static final int RPM_STORED = 5;
        public static final int MAX_VOLTAGE_COMPENSATION = 10;
        public static final int TICKS_PER_REVOLUTION = 2048;   
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
        public static final double PIVOT_DEPLOYED_TICKS = 700;

        public static final double ROLLER_SPEED = 0.7;
        public static final double FUNNEL_SPEED = 0.3;

        public static final double PIVOT_TICKS_PER_REVOLUTION = 4096;
        public static final double PIVOT_GEAR_RATIO = 10;

        public static final double MARGIN_OF_ERROR_TICKS = 80;

        public static final double P = 0.7;
        public static final double I = 0;
        public static final double D = 0;
        public static final double FF = 0.2;
        public static final double ROTATIONAL_INERTIA = 0.1;
        public static final double PIVOT_MASS = 68.03;
        public static final double PIVOT_LENGTH = 0.1;
        public static final double PIVOT_MIN_ANGLE = -Math.PI/12;
        public static final double PIVOT_MAX_ANGLE = Math.PI/2 + Math.PI/12;
    }
    public static class Drivetrain{
        //Constants
        //zero and ask mech for vals
        // public static final double kLimelightFF = 0.042;
        // public static final double kLimelightP = 0.04;

        public static final double kTrackScrubFactor = 1.0469745223;
        public static final double kTrackWidthInches = 24.2; // inches
        public static final double kTrackWidthMeters = .5883; // meters
        public static final double kWheelDiameterMeters = .158; // meters      // .1450848;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // meters
                
        public static final double kMaxVelocityMetersPerSecond = 1.5; // m/s
        public static final double kMaxAccelerationMetersPerSecondSq = 2;// m/s^2
        public static final double kDriveGearRatio = (46.0/9) * (44.0/20); // ticks
        public static final int kFalconTicksPerRotation = 2048; // ticks/rotation
                
        public static final double kDriveS = 0.166; 
        public static final double kDriveV = 2.53; 
        public static final double kDriveA = 0.311;  

        public static final double kDriveP = 0; 
        public static final double kDriveI = 0;
        public static final double kDriveD = 0;
                
        public static final double kSensitivity = 0.90;
        public static final double kWheelDeadband = 0.02;
        public static final double kThrottleDeadband = 0.02;
                
        public static final double kRobotMass = 125.0; // kg
        public static final double kRotationalInertia = 20.0;

        public static final double kAlignP = 0.0055;
        public static final double kAlignI = 0;
        public static final double kAlignD = 0;
        public static final double kAlignff = 0.0033;

        public static final double kAcceptableAlignError = 1.5; 

        public static final double kIntegralRange = 1;
    }
}