/**
 * ClimberInterface.java
 * @version 1.0
 * @since 4/16/2021
 * Interface that provides methods that every climber should have
 */

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public interface ClimberInterface 
{
    /**
     * @return true if the robot is real; false otherwise
     */
    public static boolean isReal()
    {
        return RobotBase.isReal();
    }

    /**
     * @return the motorID of the elevator
     */
    public static int getMotorID()
    {
        if(Constants.kCompBot)
            return 22;
        return 9;
    }

    /**
     * Converts ticks to meters
     * @param ticks
     * @return meters
     */
    public static double ticksToMeters(double ticks)
    {
        return ticks * Constants.Climber.DISTANCE_PER_PULSE;
    }

    /**
     * Converts meters to ticks
     * @param meters
     * @return ticks
     */
    public static double metersToTicks(double meters)
    {
        return meters / Constants.Climber.DISTANCE_PER_PULSE;
    }

    /**
     * Sets the required inversions, PID, and other necessary things for the climber motor controller
     * @param elevatorMaster
     */
    public static void set(BaseTalon elevatorMaster)
    {
        elevatorMaster.setInverted(Constants.kCompBot);
        elevatorMaster.setSelectedSensorPosition(0);
        elevatorMaster.configPeakOutputForward(1);
        elevatorMaster.configPeakOutputReverse(-1);
        elevatorMaster.config_kP(Constants.kPIDIdx, Constants.Climber.kElevatorP);
        elevatorMaster.config_kI(Constants.kPIDIdx, Constants.Climber.kElevatorI);
        elevatorMaster.config_kD(Constants.kPIDIdx, Constants.Climber.kElevatorD);
        elevatorMaster.configVoltageCompSaturation(10, Constants.kTimeoutMs);
        elevatorMaster.enableVoltageCompensation(true);
    }

    /**
     * @return true if the robot is in between top and bottom setpoints; false otherwise
     */
    public default boolean inBounds()
    {
        return this.getDistanceTicks() <= Constants.Climber.kClimbHeight;
    }

    /**
     * @return true if the robot is at the bottom
     */
    public abstract boolean atBottom();

    /**
     * @param RollingAverage average used if the robot is at the top
     * @return true if the robot is at the top
     */
    public static boolean atTop(RollingAverage heightAverage)
    {
        return Math.abs(heightAverage.getAverage() - Constants.Climber.kClimbHeight) <= Constants.Climber.ACCEPTABLE_AMOUNT;
    }

    /**
     * @return the distance in native units(ticks) of the elevator
     */
    public abstract double getDistanceTicks();
    
    /**
     * @param position the position that the robot wants to go to
     */
    public abstract void setPosition(double position);

    /**
     * @return the distance that the elevator has travelled in meters
     */
    public abstract double getDistance();

    /**
     * Zeroes the elevator to go to the minimum height
     */
    public abstract void zero();
    
    /**
     * Moves the elevator to the max height
     */
    public abstract void climb();

    /**
     * Stops the elevator
     */
    public abstract void stop();

    /**
     * The user manually controls the motor
     */
    public abstract void setMotorOutputPercent(double percent);
    
    /**
     * Simulates the limit switch
     * @throws UnsupporedOperationEception
     */
    public abstract void simulate(Servo servo) throws UnsupportedOperationException;

    public abstract double getMotorOutputPercent();
}
