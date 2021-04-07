// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utils.ClimberInterface;
import frc.robot.utils.RollingAverage;

/** Add your docs here. */
public class ClimberMethods implements ClimberInterface 
{
    public ClimberMethods(){}

    public double getDistanceTicks()
    {
        if(isReal())
        {
            return Hardware.Climber.elevatorMaster.getSelectedSensorPosition();
        }

        return Hardware.Climber.elevatorEncoder.getDistance() / Constants.Climber.DISTANCE_PER_PULSE;
    }
    
    public boolean atBottom(RollingAverage heightAverage)
    {
        if(isReal())
        {
            return Hardware.Climber.elevatorBottomLimitSwitch.get();
        }

        return Math.abs(heightAverage.getAverage() - Constants.Climber.kElevatorZero) <= Constants.Climber.ACCEPTABLE_AMOUNT;
    }

    public boolean atTop(RollingAverage heightAverage)
    {
        return Math.abs(heightAverage.getAverage() - Constants.Climber.kClimbHeight) <= Constants.Climber.ACCEPTABLE_AMOUNT;
    }

    public boolean inBounds()
    {
        return this.getDistanceTicks() <= Constants.Climber.kClimbHeight;
    }
}
