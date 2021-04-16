/**
 * ClimberManualTrigger.java
 * @version 1.0
 * @since 4/16/2021
 * Class used for calling the manual command if the trigger is pressed
 */
package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class ClimberManualTrigger extends Trigger
{
    @Override
    /**
     * @return true if the joystick is moved; false otherwise
     */
    public boolean get()
    {
        return Math.abs(Robot.getContainer().getLeft() - 0.1) > 0;
    }
}
