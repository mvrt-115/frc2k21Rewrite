// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;

/** Add your docs here. */
public class Hardware {
    //No simulation support with TalonFX
    public static class Climber
    {
        public static BaseTalon elevatorMaster;
        public static DigitalInput elevatorBottomLimitSwitch;
        public static Servo elevatorServo;
        public static TalonSRXSimCollection elevatorMasterSim;
        public static ElevatorSim elevatorSim;
        public static DCMotor elevatorMotor;
    }
}
