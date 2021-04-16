/**
 * Climber.java
 * @version 1.0
 * @since 4/16/2021
 * This climber class is used for representing and controlling an elevator that is to be used
 * on the robot. It provides various methods and elevator states(enum) that are to be used.
 */

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
  private ElevatorState currState;                          //current state of the elevator
  private SupplyCurrentLimitConfiguration currentConfig;    //limiting the current flow into the elevator
  private RollingAverage heightAverage;                     //averaging the height of the elevator

  private ClimberInterface climberMethods;
  
  private static BaseTalon elevatorMaster;
  private static Servo elevatorServo;
  private static DigitalInput elevatorBottomLimitSwitch;

  //The following are for simulation purposes only. They are set to null on a real object
  private static Encoder elevatorEncoder;
  private static EncoderSim elevatorEncoderSim;
  private static ElevatorSim elevatorSim;
  private static DCMotor elevatorMotor;

  /**
   * Enum ElevatorState that represents various states of the elevator
   * CLIMBING: going to the setpoint height (going up)
   * HOLD: holds the elevator at that state
   * ZEROING: going to the bottom (going down)
   * MANUAL_OVERRIDE: the climber is to be controlled with the joysticks
   */
  public enum ElevatorState 
  {
    CLIMBING, HOLD, ZEROING, MANUAL_OVERRIDE
  };

  private static int motorID;

  /** Creates a new Climber. */
  public Climber() 
  {
    climberMethods = new ClimberMethods();
    elevatorServo = new Servo(0);
    heightAverage = new RollingAverage(5);

    motorID = climberMethods.getMotorID();

    //for a real robot, only use a motor and a limit switch
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

  /**
   * Resets all the variables above to zero.
   */
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

  @Override
  /**
   * This method is called constantly.
   */
  public void periodic() 
  {
    double sensorPosition = climberMethods.getDistanceTicks(elevatorMaster, elevatorSim);

    switch(currState)
    {
      //Sets the servo to unratched state and moves the climber up
      case CLIMBING:
        elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        climberMethods.setPosition(elevatorMaster, elevatorEncoder, Constants.Climber.kClimbHeight);
        if(Math.abs(heightAverage.getAverage() - Constants.Climber.kClimbHeight) < 0.02)
          currState = ElevatorState.HOLD;
        break;

      //Sets the servo to unratched state and moves the climber down
      case ZEROING:
        elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        climberMethods.setPosition(elevatorMaster, elevatorEncoder, Constants.Climber.kElevatorZero);
        if(Math.abs(heightAverage.getAverage() - Constants.Climber.kElevatorZero) < 0.02)
          currState = ElevatorState.HOLD;
        break;
      
      //Sets the servo to ratched state so that the elevator would not move
      case HOLD:
        elevatorServo.setAngle(Constants.Climber.kServoRatchet);
        elevatorMaster.set(ControlMode.PercentOutput, 0.0);
        break;

      //Sets the servo to unratched state and allows the joysticks to control the robot
      //if the climber is at the full top, it automatically moves it down
      //if the climber is at the full bottom, it automatically moves it up
      //otherwise, the climber is controlled by the user
      case MANUAL_OVERRIDE:
        elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        if(sensorPosition == Constants.Climber.kClimbHeight)
        {
          elevatorMaster.set(ControlMode.PercentOutput, -0.1);  
        }
        else if(sensorPosition == Constants.Climber.kElevatorZero)
        {
          elevatorMaster.set(ControlMode.PercentOutput, 0.1);
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

  /**
   * This method is used for simulating the robot.
   */
  public void simulationPeriodic() 
  {
    super.simulationPeriodic();

    //sets the state of the limit switch
    if(elevatorSim.getPositionMeters() <= .03)
      ((DigitalInputWrapper)elevatorBottomLimitSwitch).set(true); 
    else ((DigitalInputWrapper)elevatorBottomLimitSwitch).set(false);

    //if the servo state is not ratched, then updates the climber simulation
    if(elevatorServo.getAngle() != Constants.Climber.kServoRatchet)
    {
      elevatorSim.setInput(elevatorMaster.getMotorOutputVoltage());
      elevatorSim.update(0.020);
      elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
  }

  /**
   * @return ElevatorState the current state of the elevator
   */
  public ElevatorState getElevatorState() 
  {
    return this.currState;
  }

  /**
   * @param ElevatorState the state to set the elevator
   */
  public void setElevatorState(ElevatorState state) 
  {
    this.currState = state;
  }

  /**
   * Logs information such as the height of the elevator in ticks, the servo angle, 
   * the value of the bottom limit switch, the elevator state, the motor output,
   * and if this is a simulation or not.
   */
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
