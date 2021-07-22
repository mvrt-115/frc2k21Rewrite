// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.kauailabs.navx.frc.AHRS;
/** Add your docs here. */

import frc.robot.utils.Limelight;
public class Hardware {
    public static class ExampleSubsystem {
        public static BaseTalon rightLeader;
        /** simulation */
    }
    public static Limelight limelight = new Limelight();
    public static AHRS gyro;        
}