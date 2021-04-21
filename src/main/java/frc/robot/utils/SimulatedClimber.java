/**
 * ClimberMethods.java
 * @version 1.0
 * @since 4/16/2021
 * Provides implementation for the methods defined by ClimberInterface
 */

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

//Two classes for simulation and real
public class SimulatedClimber implements ClimberInterface 
{
    //used for simulation PID
    private double lastTime, lastError, errorSum;
    private WPI_TalonSRX elevatorMaster;
    private ElevatorSim elevatorSim;
    private Encoder encoder;

    public SimulatedClimber(BaseTalon elevatorMaster, ElevatorSim elevatorSim, Encoder encoder)
    {
        lastTime = lastError = errorSum = 0;

        this.elevatorMaster = (WPI_TalonSRX)(elevatorMaster);
        this.elevatorSim = elevatorSim;
        this.encoder = encoder;
    }

    /**
     * @see ClimberInterface.java
     */
    public double getDistanceTicks()
    {
        return ClimberInterface.metersToTicks(elevatorSim.getPositionMeters());
    }

    /**
     * @see ClimberInterface.java
     */
    public double getDistance()
    {
        return encoder.getDistance();
    }

    /**
     * @see ClimberInterface.java
     */
    public void setPosition(double position)
    {
        double error = ClimberInterface.ticksToMeters(position) - getDistance();
        double currTime = Timer.getFPGATimestamp();
        double dt = currTime - lastTime;

        errorSum += error;
        double output = Constants.Climber.kElevatorP * error + Constants.Climber.kElevatorI * errorSum + Constants.Climber.kElevatorD * (error - lastError)/dt;

        elevatorMaster.set(output);
        lastError = error;
        lastTime = currTime;
    }
}
