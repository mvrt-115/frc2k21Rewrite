/**
 * ClimberMethods.java
 * @version 1.0
 * @since 4/16/2021
 * Provides implementation for the methods defined by ClimberInterface
 */

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

//Two classes for simulation and real
public class RealClimber implements ClimberInterface 
{
    private TalonFX elevatorMaster;

    public RealClimber(BaseTalon elevatorMaster) 
    {
        this.elevatorMaster = (TalonFX)(elevatorMaster);
    }

    /**
     * @see ClimberInterface.java
     */
    public double getDistanceTicks()
    {
        return elevatorMaster.getSelectedSensorPosition();
    }

    /**
     * @see ClimberInterface.java
     */
    public double getDistance()
    {
        return ClimberInterface.ticksToMeters(elevatorMaster.getSelectedSensorPosition());
    }

    /**
     * @see ClimberInterface.java
     */
    public void setPosition(double position)
    {
        elevatorMaster.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, Constants.Climber.kElevatorClimbOutput);
    }
}
