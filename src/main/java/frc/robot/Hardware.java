// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DigitalInput;


/** Hardware for all subsystems */
public class Hardware {
    /** Hardware for the Hopper class (broken into 2 sections (top and bottom)) */
    public static class Hopper {
        public static BaseTalon top;
        public static TalonSRXSimCollection topLeaderMotorControllerSim; // for simulation

        public static BaseTalon bottom;
        public static TalonSRXSimCollection bottomMotorControllerSim; // for simulation

        public static DigitalInput breakbeamMid;
        public static DigitalInput breakbeamTop;
        public static DigitalInput breakbeamBot;
    }
}
