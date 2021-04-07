// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class RollingAverage
{
    private java.util.List<Double> values;
    private int capacity;

    public RollingAverage(int capacity)
    {
        this.capacity = capacity != 0 ? capacity : 1;
        values = new java.util.ArrayList<Double>(this.capacity);
    }

    public double getAverage()
    {
        double sum = 0.0;

        for(Double value : values)
        {
            sum += value;
        }

        return values.size() == 0 ? 0 : sum / (values.size());
    }

    public void add(double element)
    {
        if(values.size() > this.capacity)
        {
            values.remove(0);
        }
        values.add(element);
    }

    public void zero()
    {
        values = new java.util.ArrayList<Double>(this.capacity);
    }

    public String toString()
    {
        return values.toString();
    }
}
