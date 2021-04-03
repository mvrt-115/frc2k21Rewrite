// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class Hardware {
    // public static class ExampleSubsystem {
    //     public static BaseTalon rightLeader;

    //     // simulation
    //     public static TalonSRXSimCollection rightMotorControllerSim;
    // }

    public static class Hopper {
        public static BaseTalon top;
        public static BaseTalon bottom;
        
        public static TalonSRXSimCollection topSim;
        public static TalonSRXSimCollection bottomSim;

        public static DigitalInput diTop;
        public static DigitalInput diMid;
        public static DigitalInput diBot;
    }
}
