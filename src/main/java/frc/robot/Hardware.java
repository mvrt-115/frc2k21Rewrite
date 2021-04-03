// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;

/** Add your docs here. */
public class Hardware {
    public static class ExampleSubsystem {
        public static BaseTalon rightLeader;

        // simulation
        public static TalonSRXSimCollection rightMotorControllerSim;
    }

    public static class Climber
    {
        public static WPI_TalonFX elevatorMaster;
        public static DigitalInput elevatorBottomLimitSwitch;
        public static Servo elevatorServo;
        
        public static class ClimberSimulation
        {
            public static DCMotor gearbox;
            public static Encoder elevatorEncoder;
            public static EncoderSim elevatorEncoderSimulation;
            public static ElevatorSim elevatorSimulation;
        }
    }
}
