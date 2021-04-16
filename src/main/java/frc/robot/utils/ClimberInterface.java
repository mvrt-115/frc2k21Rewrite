/**
 * ClimberInterface.java
 * @version 1.0
 * @since 4/16/2021
 * Interface that provides methods that every climber should have
 */

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public interface ClimberInterface 
{
    /**
     * @return true if the robot is real; false otherwise
     */
    public default boolean isReal()
    {
        return RobotBase.isReal();
    }

    /**
     * @return true if the robot is a simulation; false otherwise
     */
    public default boolean isSimulation()
    {
        return RobotBase.isSimulation();
    }

    /**
     * @param BaseTalon elevatorMaster used if the robot is real
     * @param ElevatorSim elevatorSim used if the robot is a simulation
     * @return the distance in native units(ticks) of the elevator
     */
    public abstract double getDistanceTicks(BaseTalon elevatorMaster, ElevatorSim elevatorSim);
   
    /**
     * @param DigitalInput elevatorBottomLimitSwitch used if the robot is real
     * @param RollingAverage average used if the robot is a simulation
     * @return true if the robot is at the bottom
     */
    public abstract boolean atBottom(DigitalInput elevatorBottomLimitSwitch, RollingAverage average);
    
    /**
     * @param RollingAverage average used if the robot is at the top
     * @return true if the robot is at the top
     */
    public abstract boolean atTop(RollingAverage average);

    /**
     * @param BaseTalon elevatorMaster used if the robot is real
     * @param ElevatorSim used if the elevator is a simulation 
     * @return true if the robot is in between top and bottom setpoints; false otherwise
     */
    public abstract boolean inBounds(BaseTalon elevatorMaster, ElevatorSim elevatorSim);

    /**
     * @param BaseTalon elevatorMaster used if the robot is real
     * @param encoder used if the robot is a simulation
     * @param position the position that the robot wants to go to
     */
    public abstract void setPosition(BaseTalon elevatorMaster, Encoder encoder, double position);

    /**
     * @return the motorID of the elevator
     */
    public abstract int getMotorID();

    /**
     * @param elevatorMaster used if the robot is real
     * @param encoder used if the robot is a simulation
     * @return the distance that the elevator has travelled in meters
     */
    public abstract double getDistance(BaseTalon elevatorMaster, Encoder encoder);

    /**
     * Converts ticks to meters
     * @param ticks
     * @return meters
     */
    public abstract double ticksToMeters(double ticks);

    /**
     * Converts meters to ticks
     * @param meters
     * @return ticks
     */
    public abstract double metersToTicks(double meters);
}
