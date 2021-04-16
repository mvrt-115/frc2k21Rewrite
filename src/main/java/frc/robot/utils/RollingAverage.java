/**
 * RollingAverage.java
 * @version 1.0
 * @since 4/16/21
 * Returns the average values
 */
package frc.robot.utils;

/** Add your docs here. */
public class RollingAverage
{
    private java.util.List<Double> values;
    private int capacity;

    /**
     * @param int capacity of the rolling average i.e. how much of the input added do you want
     * to be included in the average
     */
    public RollingAverage(int capacity)
    {
        this.capacity = capacity != 0 ? capacity : 1;
        values = new java.util.ArrayList<Double>(this.capacity);
    }

    /**
     * @return the average values of the last capacity elements
     */
    public double getAverage()
    {
        double sum = 0.0;

        for(Double value : values)
        {
            sum += value;
        }

        return values.size() == 0 ? 0 : sum / (values.size());
    }

    /**
     * @param element the element to add
     * If the size of the number of elements becomes bigger than the capacity, it removes the first one
     */
    public void add(double element)
    {
        if(values.size() > this.capacity)
        {
            values.remove(0);
        }
        values.add(element);
    }

    /**
     * Zeroes everything out and creates a new rolling average
     */
    public void zero()
    {
        values = new java.util.ArrayList<Double>(this.capacity);
    }

    /**
     * @return String all the last capacity values as a string
     */
    public String toString()
    {
        return values.toString();
    }
}
