// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.utils.ClimberInterface;
import frc.robot.utils.RollingAverage;

/** Add your docs here. */
public class ClimberMethods implements ClimberInterface 
{
    double lastTime, lastError, errorSum;

    public ClimberMethods()
    {
        lastTime = lastError = errorSum = 0;
    }

    public double getDistanceTicks(BaseTalon elevatorMaster, ElevatorSim elevatorSim)
    {
        if(isReal())
            return elevatorMaster.getSelectedSensorPosition();
        return metersToTicks(elevatorSim.getPositionMeters());
    }

    public double getDistance(BaseTalon elevatorMaster, Encoder encoder)
    {
        if(isReal())
            return ticksToMeters(elevatorMaster.getSelectedSensorPosition());
        return encoder.getDistance();
    }
    
    public boolean atBottom(DigitalInput elevatorBottomLimitSwitch, RollingAverage heightAverage)
    {
        if(isReal())
        {
            return elevatorBottomLimitSwitch.get();
        }

        return Math.abs(heightAverage.getAverage() - Constants.Climber.kElevatorZero) <= Constants.Climber.ACCEPTABLE_AMOUNT;
    }

    public boolean atTop(RollingAverage heightAverage)
    {
        return Math.abs(heightAverage.getAverage() - Constants.Climber.kClimbHeight) <= Constants.Climber.ACCEPTABLE_AMOUNT;
    }

    public double ticksToMeters(double ticks)
    {
        return ticks * Constants.Climber.DISTANCE_PER_PULSE;
    }

    public double metersToTicks(double meters)
    {
        return meters / Constants.Climber.DISTANCE_PER_PULSE;
    }

    public boolean inBounds(BaseTalon elevatorMaster, ElevatorSim elevatorSim)
    {
        return this.getDistanceTicks(elevatorMaster, elevatorSim) <= Constants.Climber.kClimbHeight;
    }

    public void setPosition(BaseTalon elevatorMaster, Encoder encoder, double position)
    {
        if(this.isReal())
        {
            ((TalonFX)(elevatorMaster)).set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, Constants.Climber.kElevatorClimbOutput);
        }
        else
        {
            double error = ticksToMeters(position) - getDistance(elevatorMaster, encoder);
            double currTime = Timer.getFPGATimestamp();
            double dt = currTime - lastTime;

            errorSum += error;
            double output = Constants.Climber.kElevatorP * error + Constants.Climber.kElevatorI * errorSum + Constants.Climber.kElevatorD * (error - lastError)/dt;

            ((WPI_TalonSRX)(elevatorMaster)).set(output);
            lastError = error;
            lastTime = currTime;
        }
    }

    public int getMotorID()
    {
        if(Constants.kCompBot)
            return 22;
        return 9;
    }
}
