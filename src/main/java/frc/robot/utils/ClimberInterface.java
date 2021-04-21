/**
 * ClimberInterface.java
 * @version 1.0
 * @since 4/16/2021
 * Interface that provides methods that every climber should have
 */

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

//make it a subsystem interface
//move DigitalInput and RollingAverage here
//parameters that are passed are not really inherent to the climber itself
//maybe even move the hardware into the classes
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
     * @param BaseTalon elevatorMaster used if the robot is real
     * @param ElevatorSim used if the elevator is a simulation 
     * @return true if the robot is in between top and bottom setpoints; false otherwise
     */
    public default boolean inBounds(BaseTalon elevatorMaster, ElevatorSim elevatorSim)
    {
        return this.getDistanceTicks() <= Constants.Climber.kClimbHeight;
    }

    /**
     * @param DigitalInput elevatorBottomLimitSwitch used if the robot is real
     * @param RollingAverage average used if the robot is a simulation
     * @return true if the robot is at the bottom
     */
    public static boolean atBottom(DigitalInput elevatorBottomLimitSwitch)
    {
        return elevatorBottomLimitSwitch.get();
    }

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
}
