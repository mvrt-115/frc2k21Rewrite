// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
import frc.robot.ClimberMethods;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.DigitalInputWrapper;
import frc.robot.utils.ClimberInterface;
import frc.robot.utils.RollingAverage;

public class Climber extends SubsystemBase 
{
  private ElevatorState currState;
  private SupplyCurrentLimitConfiguration currentConfig;
  private RollingAverage heightAverage;

  private ClimberInterface climberMethods;
  
  private static BaseTalon elevatorMaster;
  private static Servo elevatorServo;
  private static Encoder elevatorEncoder;
  private static EncoderSim elevatorEncoderSim;
  private static ElevatorSim elevatorSim;
  private static DCMotor elevatorMotor;
  private static DigitalInput elevatorBottomLimitSwitch;

  public enum ElevatorState 
  {
    CLIMBING, HOLD, ZEROING, ZEROED, MANUAL_OVERRIDE
  };

  private static int motorID;

  /** Creates a new Climber. */
  public Climber() 
  {
    climberMethods = new ClimberMethods();
    elevatorServo = new Servo(0);
    heightAverage = new RollingAverage(5);

    motorID = climberMethods.getMotorID();

    if (RobotBase.isReal()) 
    {
      elevatorMaster = new TalonFX(motorID);
      elevatorBottomLimitSwitch = new DigitalInput(2);
    }
    else
    {
      elevatorMotor = DCMotor.getVex775Pro(1);
      elevatorSim = new ElevatorSim
      (
        elevatorMotor, 
        Constants.Climber.GEAR_REDUCTION,
        Constants.Climber.CARRIAGE_MASS, 
        Constants.Climber.PULLEY_RADIUS, 
        Constants.Climber.MIN_HEIGHT,
        Constants.Climber.MAX_HEIGHT, 
        null
      );

      elevatorMaster = new WPI_TalonSRX(motorID);
      elevatorEncoder = new Encoder(0, 1);
      elevatorEncoderSim = new EncoderSim(elevatorEncoder);
      elevatorBottomLimitSwitch = new DigitalInputWrapper(2);
    }

    reset();
  }

  private void reset()
  {
    currState = ElevatorState.HOLD;

    heightAverage.zero();

    if (RobotBase.isReal()) 
    {
      elevatorMaster.configFactoryDefault();
      
      ((TalonFX)(elevatorMaster)).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx, Constants.kTimeoutMs);

      currentConfig = new SupplyCurrentLimitConfiguration(true, 40, 1.3, 30);
      ((TalonFX) (elevatorMaster)).configSupplyCurrentLimit(currentConfig);
      elevatorBottomLimitSwitch = new DigitalInput(2);
    } 
    else 
    {
      elevatorMaster.configFactoryDefault();
      elevatorEncoder.reset();
      elevatorEncoder.setDistancePerPulse(Constants.Climber.DISTANCE_PER_PULSE);
    }
    
    elevatorMaster.setInverted(Constants.kCompBot);
    elevatorMaster.setSelectedSensorPosition(0);
    elevatorMaster.configPeakOutputForward(1);
    elevatorMaster.configPeakOutputReverse(-1);
    elevatorMaster.config_kP(Constants.kPIDIdx, Constants.Climber.kElevatorP);
    elevatorMaster.config_kI(Constants.kPIDIdx, Constants.Climber.kElevatorI);
    elevatorMaster.config_kD(Constants.kPIDIdx, Constants.Climber.kElevatorD);
    elevatorMaster.configVoltageCompSaturation(10, Constants.kTimeoutMs);
    elevatorMaster.enableVoltageCompensation(true);
  }

  public void setPIDConstants(int kPIDIdx, double p, double i, double d)
  {
    elevatorMaster.config_kP(kPIDIdx, p);
    elevatorMaster.config_kI(kPIDIdx, i);
    elevatorMaster.config_kD(kPIDIdx, d);
  }

  @Override
  public void periodic() 
  {
    double sensorPosition = climberMethods.getDistanceTicks(elevatorMaster, elevatorSim);

    switch(currState)
    {
      case CLIMBING:
        elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        climberMethods.setPosition(elevatorMaster, elevatorEncoder, Constants.Climber.kClimbHeight);
        if(Math.abs(heightAverage.getAverage() - Constants.Climber.kClimbHeight) < 0.02)
          currState = ElevatorState.HOLD;
        break;

      case ZEROED:
        currState = ElevatorState.HOLD;
        break;

      case ZEROING:
        elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        climberMethods.setPosition(elevatorMaster, elevatorEncoder, Constants.Climber.kElevatorZero);
        if(Math.abs(heightAverage.getAverage() - Constants.Climber.kElevatorZero) < 0.02)
          currState = ElevatorState.ZEROED;
        break;
      
      case HOLD:
        elevatorServo.setAngle(Constants.Climber.kServoRatchet);
        elevatorMaster.set(ControlMode.PercentOutput, 0.0);
        break;

      case MANUAL_OVERRIDE:
        elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        if(sensorPosition == Constants.Climber.kClimbHeight)
        {
          elevatorMaster.set(ControlMode.PercentOutput, 0.0);  
        }
        else if(sensorPosition == Constants.Climber.kElevatorZero)
        {
          elevatorMaster.set(ControlMode.PercentOutput, 0.5);
        }
        else
        {
          elevatorMaster.set(ControlMode.PercentOutput, Robot.getContainer().getLeft());
        }
        break;
    }

    heightAverage.add(sensorPosition);
    log();
  }

  public void simulationPeriodic() 
  {
    super.simulationPeriodic();
    if(elevatorSim.getPositionMeters() <= .03)
      ((DigitalInputWrapper)elevatorBottomLimitSwitch).set(true); 
    else ((DigitalInputWrapper)elevatorBottomLimitSwitch).set(false);

    if(elevatorServo.getAngle() != Constants.Climber.kServoRatchet)
    {
      elevatorSim.setInput(elevatorMaster.getMotorOutputVoltage());
      elevatorSim.update(0.020);
      elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
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
    SmartDashboard.putNumber("Height of Elevator in ticks", climberMethods.getDistanceTicks(elevatorMaster, elevatorSim));
    SmartDashboard.putNumber("Servo Angle", elevatorServo.getAngle());
    SmartDashboard.putBoolean("Bottom Limit Switch Value of Elevator", elevatorBottomLimitSwitch.get());
    SmartDashboard.putString("Elevator State", this.getElevatorState().toString());
    SmartDashboard.putNumber("Motor Output[-1, 1]", elevatorMaster.getMotorOutputPercent());
    SmartDashboard.putBoolean("Simulating?", !climberMethods.isReal());
  }
}
