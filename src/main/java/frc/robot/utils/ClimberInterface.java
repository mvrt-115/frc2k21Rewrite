// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;

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

    public abstract double getDistanceTicks();
    public abstract boolean atBottom(RollingAverage average);
    public abstract boolean atTop(RollingAverage average);
    public abstract boolean inBounds();
}
