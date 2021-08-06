package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickTriggerButton extends Trigger {
    private int axis;
    private Joystick joystick;

    public JoystickTriggerButton(Joystick joystick, int axis) {
        super();
        this.joystick = joystick;
        this.axis = axis;
    }

    public boolean get() {
        double value = joystick.getRawAxis(axis); 
        
        if(value > 0.2) 
            return true;
        else
            return false;
    }
}
