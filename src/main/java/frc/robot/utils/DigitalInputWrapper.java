/**
 * DigitalInputWrapper.java
 * @version 1.0
 * @since 4/16/2021
 * This class is to simulate a limit switch
 */
package frc.robot.utils;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class DigitalInputWrapper extends DigitalInput
{
    private SimBoolean simSensor;
    private SimDevice simDigitalInput;

    /**
     * @param channel the limit switch chanell
     */
    public DigitalInputWrapper(int channel)
    {
        super(channel);

        simDigitalInput = SimDevice.create("Digital Input", channel);

        if(simDigitalInput != null)
        {
            simSensor = simDigitalInput.createBoolean("Sensor Reading", Direction.kInput, true);
        }
    }

    /**
     * Returns if the limit switch is triggered or not
     */
    public boolean get()
    {
        if(simDigitalInput != null)
            return simSensor.get();
        return super.get();
    }

    /**
     * @param boolean val to set the current limit switch
     */
    public void set(boolean val)
    {
        if(simDigitalInput != null)
            simSensor.set(val);
    }
}
