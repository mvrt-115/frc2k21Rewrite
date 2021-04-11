// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimberMethods;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.utils.ClimberInterface;
import frc.robot.utils.RollingAverage;

public class Climber extends SubsystemBase 
{
  private ElevatorState currState;
  private SupplyCurrentLimitConfiguration currentConfig;
  private RollingAverage heightAverage;

  private ClimberInterface climberMethods;

  public enum ElevatorState 
  {
    CLIMBING, HOLD, ZEROING, MANUAL_OVERRIDE
  };

  private static int motorID;

  /** Creates a new Climber. */
  public Climber() 
  {
    climberMethods = new ClimberMethods();
    currState = ElevatorState.HOLD;

    Hardware.Climber.elevatorBottomLimitSwitch = new DigitalInput(2);
    Hardware.Climber.elevatorServo = new Servo(0);

    heightAverage = new RollingAverage(5);
    heightAverage.zero();

    motorID = climberMethods.getMotorID();

    if (RobotBase.isReal()) 
    {
      Hardware.Climber.elevatorMaster = new TalonFX(motorID);

      Hardware.Climber.elevatorMaster.configFactoryDefault();
      
      ((TalonFX)(Hardware.Climber.elevatorMaster)).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx, Constants.kTimeoutMs);

      currentConfig = new SupplyCurrentLimitConfiguration(true, 40, 1.3, 30);
      ((TalonFX) (Hardware.Climber.elevatorMaster)).configSupplyCurrentLimit(currentConfig);
    } 
    else 
    {
      Hardware.Climber.elevatorMotor = DCMotor.getVex775Pro(1);
      Hardware.Climber.elevatorSim = new ElevatorSim
      (
        Hardware.Climber.elevatorMotor, 
        Constants.Climber.GEAR_REDUCTION,
        Constants.Climber.CARRIAGE_MASS, 
        Constants.Climber.PULLEY_RADIUS, 
        Constants.Climber.MIN_HEIGHT,
        Constants.Climber.MAX_HEIGHT, 
        null
      );

      Hardware.Climber.elevatorMaster = new WPI_TalonSRX(motorID);
      Hardware.Climber.elevatorMaster.configFactoryDefault();
      Hardware.Climber.elevatorMasterSim = ((WPI_TalonSRX) (Hardware.Climber.elevatorMaster)).getSimCollection();
    }
    
    Hardware.Climber.elevatorMaster.setInverted(Constants.kCompBot);
    Hardware.Climber.elevatorMaster.setSelectedSensorPosition(0);
    Hardware.Climber.elevatorMaster.configPeakOutputForward(1);
    Hardware.Climber.elevatorMaster.configPeakOutputReverse(-1);
    Hardware.Climber.elevatorMaster.config_kP(Constants.kPIDIdx, Constants.Climber.kElevatorP);
    Hardware.Climber.elevatorMaster.config_kI(Constants.kPIDIdx, Constants.Climber.kElevatorI);
    Hardware.Climber.elevatorMaster.config_kD(Constants.kPIDIdx, Constants.Climber.kElevatorD);
  }

  @Override
  public void periodic() 
  {
    double sensorPosition = climberMethods.getDistanceTicks();
    switch (currState) 
    {
      case ZEROING:
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.Position, Constants.Climber.kElevatorZero,
            DemandType.ArbitraryFeedForward, Constants.Climber.kElevatorClimbOutput);
        if (climberMethods.atBottom(heightAverage))
          currState = ElevatorState.HOLD;
        break;

      case CLIMBING:
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.MotionMagic, Constants.Climber.kClimbHeight,
            DemandType.ArbitraryFeedForward, Constants.Climber.kElevatorClimbOutput);
       if (climberMethods.atTop(heightAverage))
          currState = ElevatorState.HOLD;
        break;

      case HOLD:
        Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, 0.0);
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoRatchet);
        break;

      case MANUAL_OVERRIDE:
        Hardware.Climber.elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, Robot.getContainer().getLeft());
        if (climberMethods.inBounds())
          Hardware.Climber.elevatorMaster.set(ControlMode.PercentOutput, 0);
        break;
    }
    heightAverage.add(sensorPosition);
    log();
  }

  public void simulationPeriodic() 
  {
    super.simulationPeriodic();
    /*
     * if(H.getPositionMeters() <= .03) // 3 CM is the approx threshold for the
     * limit Switch bottomLimitSwitch.set(true); else bottomLimitSwitch.set(false);
     */

    if(Hardware.Climber.elevatorServo.getAngle() != Constants.Climber.kServoRatchet)
    {
      Hardware.Climber.elevatorSim.setInput(Hardware.Climber.elevatorMaster.getMotorOutputVoltage());
      Hardware.Climber.elevatorSim.update(0.02);

      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(Hardware.Climber.elevatorSim.getCurrentDrawAmps()));

      Hardware.Climber.elevatorMasterSim.setQuadratureRawPosition((int)(Hardware.Climber.elevatorSim.getPositionMeters() / Constants.Climber.DISTANCE_PER_PULSE));
      Hardware.Climber.elevatorMasterSim.setQuadratureVelocity((int)(Hardware.Climber.elevatorSim.getVelocityMetersPerSecond()/Constants.Climber.DISTANCE_PER_PULSE));

      Hardware.Climber.elevatorMasterSim.setBusVoltage(RoboRioSim.getVInVoltage());
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
    SmartDashboard.putNumber("Current in Elevator", Hardware.Climber.elevatorSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Height of Elevator in ticks", climberMethods.getDistanceTicks());
    SmartDashboard.putNumber("Servo Angle", Hardware.Climber.elevatorServo.getAngle());
    SmartDashboard.putBoolean("Bottom Limit Switch Value of Elevator", Hardware.Climber.elevatorBottomLimitSwitch.get());
    SmartDashboard.putString("Elevator State", this.getElevatorState().toString());
    SmartDashboard.putNumber("Motor Output[-1, 1]", Hardware.Climber.elevatorMaster.getMotorOutputPercent());
  }
}
