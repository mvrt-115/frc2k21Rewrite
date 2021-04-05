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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;
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
    Hardware.Climber.elevatorMaster.configSupplyCurrentLimit(currentConfig);

    currState = ElevatorState.ZEROED;

    Hardware.Climber.elevatorMaster.configPeakOutputForward(1);
    Hardware.Climber.elevatorMaster.configPeakOutputReverse(-1);

    Hardware.Climber.elevatorMaster.config_kP(Constants.kPIDIdx, Constants.Climber.kElevatorP);
    Hardware.Climber.elevatorMaster.config_kI(Constants.kPIDIdx, Constants.Climber.kElevatorI);
    Hardware.Climber.elevatorMaster.config_kD(Constants.kPIDIdx, Constants.Climber.kElevatorD);

    Hardware.Climber.elevatorBottomLimitSwitch = new DigitalInput(2);

    heightAverage = new RollingAverage(5);

    if(!RobotBase.isReal())
    {
      Hardware.Climber.gearbox = DCMotor.getFalcon500(1);
      Hardware.Climber.elevatorSimulation = new ElevatorSim
      (
        Hardware.Climber.gearbox, 
        Constants.Climber.GEAR_REDUCTION, 
        Constants.Climber.CARRIAGE_MASS, 
        Constants.Climber.PULLEY_RADIUS, 
        Constants.Climber.MIN_HEIGHT, 
        Constants.Climber.MAX_HEIGHT,
        null
      );
      Hardware.Climber.elevatorEncoder = new Encoder(Constants.Climber.CHANNEL_A, Constants.Climber.CHANNEL_B);
      Hardware.Climber.elevatorEncoder.reset();
      Hardware.Climber.elevatorEncoder.setDistancePerPulse(Constants.Climber.DISTANCE_PER_PULSE);
      Hardware.Climber.elevatorEncoderSimulation = new EncoderSim(Hardware.Climber.elevatorEncoder);
    }
  }

  @Override
  public void periodic() 
  {
    double sensorPosition = 0;
    if(RobotBase.isReal())
    {
      sensorPosition = Hardware.Climber.elevatorMaster.getSelectedSensorPosition();
    }
    else
    {
      sensorPosition = Hardware.Climber.elevatorEncoder.getDistance() / Constants.Climber.DISTANCE_PER_PULSE;
    }

    switch(currState)
    {
      case ZEROED:
        currState = ElevatorState.HOLD;
        break;
      case ZEROING:
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.Position, Constants.Climber.kElevatorZero, DemandType.ArbitraryFeedForward, Constants.Climber.kElevatorClimbOutput);
        if(RobotBase.isReal() && Hardware.Climber.elevatorBottomLimitSwitch.get() || Math.abs(heightAverage.getAverage() - Constants.Climber.kElevatorZero) <= 0.2)
          currState = ElevatorState.ZEROED;
        break;
      case CLIMBING:
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.Position, Constants.Climber.kClimbHeight, DemandType.ArbitraryFeedForward, 0);
        /*if(Math.abs(Constants.Climber.kClimbHeight - heightAverage.getAverage()) <= 0.2)
          currState = ElevatorState.HOLD;*/
        break;
      case HOLD:
        Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, 0.0);
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoRatchet);
        break;
      case MANUAL_OVERRIDE:
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, Robot.getContainer().getLeft());
        if(sensorPosition > Constants.Climber.kClimbHeight || 
          Hardware.Climber.elevatorBottomLimitSwitch.get())
          Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, 0);
    }
    heightAverage.add(sensorPosition);

    SmartDashboard.putNumber("Elevator sensor position", sensorPosition);
    SmartDashboard.putString("Elevator State", currState.toString());
    log();
  }

  public void simulationPeriodic()
  {
    super.simulationPeriodic();
    Hardware.Climber.elevatorSimulation.setInput(Hardware.Climber.elevatorMaster.getMotorOutputVoltage());

    //if(Hardware.Climber.elevatorServo.getAngle() != Constants.Climber.kServoRatchet)
    Hardware.Climber.elevatorSimulation.update(0.020);

    Hardware.Climber.elevatorEncoderSimulation.setDistance(Hardware.Climber.elevatorSimulation.getPositionMeters());

    RoboRioSim.setVInCurrent(BatterySim.calculateDefaultBatteryLoadedVoltage(Hardware.Climber.elevatorSimulation.getCurrentDrawAmps()));
  }
  
  public ElevatorState getElevatorState()
  {
    return this.currState;
  }

  public void setElevatorState(ElevatorState state)
  {
    this.currState = state;
  }

  public void log()
  {
    SmartDashboard.putNumber("Current in Elevator", Hardware.Climber.elevatorMaster.getSupplyCurrent());
    SmartDashboard.putNumber("Height of Elevator in ticks", RobotBase.isReal() ? Hardware.Climber.elevatorMaster.getSelectedSensorPosition() : Hardware.Climber.elevatorEncoder.getDistance()/Hardware.Climber.elevatorEncoder.getDistancePerPulse());
    SmartDashboard.putNumber("Speed in ticks/100ms of Elevator", RobotBase.isReal() ? Hardware.Climber.elevatorMaster.getSelectedSensorVelocity() : -1);
    SmartDashboard.putNumber("Servo Angle", Hardware.Climber.elevatorServo.getAngle());
    SmartDashboard.putBoolean("Bottom Limit Switch Value of Elevator", Hardware.Climber.elevatorBottomLimitSwitch.get());
  }
}
