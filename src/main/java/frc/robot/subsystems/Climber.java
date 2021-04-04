// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.utils.RollingAverage;

public class Climber extends SubsystemBase {
  private ElevatorState currState;
  private SupplyCurrentLimitConfiguration currentConfig;
  private RollingAverage heightAverage;

  // Climbing = going to the height
  // Hold = prevents the evelvator from moving by rotating the servo
  // Zeroing = going down
  public enum ElevatorState {
    ZEROED, CLIMBING, HOLD, ZEROING, MANUAL_OVERRIDE
  };

  /** Creates a new Climber. */
  public Climber() {
    if (Constants.kCompBot) {
      Hardware.Climber.elevatorMaster = new TalonFX(22);
    } else {
      Hardware.Climber.elevatorMaster = new TalonFX(9);
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

    Hardware.Climber.elevatorBottomLimitSwitch = new DigitalInput(2);

    heightAverage = new RollingAverage(5);
  }

  @Override
  public void periodic() 
  {
    switch(currState)
    {
      case ZEROED:
        Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, 0.0);
        currState = ElevatorState.HOLD;
        break;
      case ZEROING:
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.Position, Constants.Climber.kElevatorZero, DemandType.ArbitraryFeedForward, Constants.Climber.kElevatorClimbOutput);
        if(Hardware.Climber.elevatorBottomLimitSwitch.get())
          currState = ElevatorState.ZEROED;
        break;
      case CLIMBING:
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.Position, Constants.Climber.kClimbHeight, DemandType.ArbitraryFeedForward, Constants.Climber.kElevatorClimbOutput);
        if(Math.abs(Hardware.Climber.elevatorMaster.getSelectedSensorPosition() - heightAverage.getAverage()) >= 0.2)
          currState = ElevatorState.HOLD;
        break;
      case HOLD:
        Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, 0.0);
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoRatchet);
        break;
      case MANUAL_OVERRIDE:
        //TODO
    }

    heightAverage.add(Hardware.Climber.elevatorMaster.getSelectedSensorPosition());
  }
}
