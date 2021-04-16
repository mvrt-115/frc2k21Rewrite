// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public interface ClimberInterface 
{
    public default boolean isReal()
    {
        return RobotBase.isReal();
    }

    public default boolean isSimulation()
    {
        return RobotBase.isSimulation();
    }

    public abstract double getDistanceTicks(BaseTalon elevatorMaster, ElevatorSim elevatorSim);
    public abstract boolean atBottom(DigitalInput elevatorBottomLimitSwitch, RollingAverage average);
    public abstract boolean atTop(RollingAverage average);
    public abstract boolean inBounds(BaseTalon elevatorMaster, ElevatorSim elevatorSim);
    public abstract void setPosition(BaseTalon elevatorMaster, Encoder encoder, double position);
    public abstract int getMotorID();
    public abstract double getDistance(BaseTalon elevatorMaster, Encoder encoder);
    public abstract double ticksToMeters(double ticks);
    public abstract double metersToTicks(double meters);
}
