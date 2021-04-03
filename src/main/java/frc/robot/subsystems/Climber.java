// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;

public class Climber extends SubsystemBase {
  private ElevatorState currState;
  private SupplyCurrentLimitConfiguration currentConfig;

  // Setpoint = going to a specific height
  // Climbing = go to the height and then rotate the servo
  // Hold = prevents the evelvator from moving by rotating the servo
  // Zeroing = going Down
  public enum ElevatorState {
    ZEROED, SETPOINT, CLIMBING, HOLD, ZEROING, MANUAL_OVERRIDE
  };

  /** Creates a new Climber. */
  public Climber() {
    if (Constants.kCompBot) {
      Hardware.Climber.elevatorMaster = new WPI_TalonFX(22);
    } else {
      Hardware.Climber.elevatorMaster = new WPI_TalonFX(9);
    }

    Hardware.Climber.elevatorServo = new Servo(0);
    Hardware.Climber.elevatorMaster.configFactoryDefault();
    Hardware.Climber.elevatorMaster.setInverted(Constants.kCompBot);

    Hardware.Climber.elevatorMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.kPIDIdx, Constants.kTimeoutMs);
    Hardware.Climber.elevatorMaster.setSelectedSensorPosition(0);

    currentConfig = new SupplyCurrentLimitConfiguration(true, 40, 1.3, 30);

    // Hardware.bottomHopper.configSupplyCurrentLimit(currentConfig,
    // Constants.kTimeoutMs);
    // Hardware.topHopper.configSupplyCurrentLimit(currentConfig,
    // Constants.kTimeoutMs);

    currState = ElevatorState.ZEROED;

    Hardware.Climber.elevatorMaster.configPeakOutputForward(1);
    Hardware.Climber.elevatorMaster.configPeakOutputReverse(-1);

    Hardware.Climber.elevatorMaster.config_kP(Constants.kPIDIdx, Constants.Climber.kElevatorP);
    Hardware.Climber.elevatorMaster.config_kI(Constants.kPIDIdx, Constants.Climber.kElevatorI);
    Hardware.Climber.elevatorMaster.config_kD(Constants.kPIDIdx, Constants.Climber.kElevatorD);

    Hardware.Climber.ClimberSimulation.gearbox = DCMotor.getFalcon500(1);
    Hardware.Climber.ClimberSimulation.elevatorSimulation = new ElevatorSim(Hardware.Climber.ClimberSimulation.gearbox,
        Constants.Climber.ClimberSimulation.GEAR_REDUCTION, Constants.Climber.ClimberSimulation.CARRIAGE_MASS,
        Constants.Climber.ClimberSimulation.PULLEY_RADIUS, Constants.Climber.ClimberSimulation.MIN_HEIGHT,
        Constants.Climber.ClimberSimulation.MAX_HEIGHT);

    Hardware.Climber.ClimberSimulation.elevatorEncoder = new Encoder(0, 1);
    Hardware.Climber.ClimberSimulation.elevatorEncoder.reset();
    Hardware.Climber.ClimberSimulation.elevatorEncoder
        .setDistancePerPulse(Constants.Climber.ClimberSimulation.Distance_PER_PULSE);
    Hardware.Climber.ClimberSimulation.elevatorEncoderSimulation = new EncoderSim(
        Hardware.Climber.ClimberSimulation.elevatorEncoder);

    Hardware.Climber.elevatorBottomLimitSwitch = new DigitalInput(2);
  }

  @Override
  public void periodic() 
  {
    switch(currState)
    {
      case ZEROED:
        SmartDashboard.putString("Climber State", "ZEROED");
        break;
      case SETPOINT:
        Hardware.Climber.elevatorMaster.set(ControlMode.Position, Constants.Climber.kClimbHeight, DemandType.ArbitraryFeedForward, Constants.Climber.kElevatorHoldOutput);
        SmartDashboard.putString("Climber State", "SETPOINT");
        break;
      case CLIMBING:
        Hardware.Climber.elevatorServo.set(Constants.Climber.kServoUnRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, Constants.Climber.kElevatorClimbOutput);
        if(Hardware.Climber.elevatorMaster.getSelectedSensorPosition() < Constants.Climber.kClimbTicks)
          currState = ElevatorState.HOLD;
        SmartDashboard.putString("Climber State", "CLIMBING");
        break;
      case HOLD:
        Hardware.Climber.elevatorServo.set(Constants.Climber.kServoRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, 0);
        SmartDashboard.putString("Climber State", "HOLD");
        break;
      case ZEROING:
        Hardware.Climber.elevatorMaster.set(ControlMode.Position, Constants.Climber.kElevatorZero, DemandType.ArbitraryFeedForward,
          Constants.Climber.kElevatorHoldOutput);
        if(Hardware.Climber.elevatorMaster.getSelectedSensorPosition() < 4000 || Hardware.Climber.elevatorBottomLimitSwitch.get())
          currState = ElevatorState.ZEROED;
      case MANUAL_OVERRIDE:
        //INSERT JOYSTICK CODE HERE
        break;
    }
  }
}
