/**
 * Climber.java
 * @version 1.0
 * @since 4/16/2021
 * This climber class is used for representing and controlling an elevator that is to be used
 * on the robot. It provides various methods and elevator states(enum) that are to be used.
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.RealClimber;
import frc.robot.utils.ClimberInterface;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.SimulatedClimber;

public class Climber extends SubsystemBase 
{
  private ElevatorState currState;          //current state of the elevator
  private RollingAverage heightAverage;     //averaging the height of the elevator

  private ClimberInterface climberMethods;  //based on real or simulation, does different things
  
  private static Servo elevatorServo;       //servo is used for ratcheting or unratheting the elevator
  private static int motorID;               //motorID of the motor

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

  /** Creates a new Climber. */
  public Climber() 
  {
    elevatorServo = new Servo(0);
    heightAverage = new RollingAverage(5);

    motorID = ClimberInterface.getMotorID();

    if (RobotBase.isReal()) 
      climberMethods = new RealClimber(motorID, 2);
    else
      climberMethods = new SimulatedClimber(motorID, 2);

    currState = ElevatorState.HOLD;

    heightAverage.zero();
  }

  @Override
  /**
   * This method is called constantly.
   */
  public void periodic() 
  {
    double sensorPosition = climberMethods.getDistanceTicks();

    //Have methods into the individual climber classes to make it more readable 
    switch(currState)
    {
      //Sets the servo to unratched state and moves the climber up
      case CLIMBING:
        elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        climberMethods.climb();
        if(Math.abs(heightAverage.getAverage() - Constants.Climber.kClimbHeight) < 0.02)
          currState = ElevatorState.HOLD;
        break;

      //Sets the servo to unratched state and moves the climber down
      case ZEROING:
        elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);
        climberMethods.zero();
        if(Math.abs(heightAverage.getAverage() - Constants.Climber.kElevatorZero) < 0.02)
          currState = ElevatorState.HOLD;
        break;
      
      //Sets the servo to ratched state so that the elevator would not move
      case HOLD:
        elevatorServo.setAngle(Constants.Climber.kServoRatchet);
        climberMethods.stop();
        break;

      //Sets the servo to unratched state and allows the joysticks to control the robot
      //if the climber is at the full top, it automatically moves it down
      //if the climber is at the full bottom, it automatically moves it up
      //otherwise, the climber is controlled by the user
      case MANUAL_OVERRIDE:
        elevatorServo.setAngle(Constants.Climber.kServoUnRatchet);

        if(sensorPosition == Constants.Climber.kClimbHeight)
          climberMethods.setMotorOutputPercent(-0.1);
        
        else if(sensorPosition == Constants.Climber.kElevatorZero)
          climberMethods.setMotorOutputPercent(0.1);
        
        else
          climberMethods.setMotorOutputPercent(Robot.getContainer().getLeft());
        
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

    //Never goes in the catch statement because this is a simulation
    try
    {
      climberMethods.simulate(elevatorServo);
    } 
    catch(UnsupportedOperationException e) 
    {
      System.err.println("Not sure how to simulate a real robots :) :) :)");
      System.exit(1);
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
    SmartDashboard.putNumber("Height of Elevator in ticks", climberMethods.getDistanceTicks());
    SmartDashboard.putNumber("Servo Angle", elevatorServo.getAngle());
    SmartDashboard.putBoolean("Bottom Limit Switch Value of Elevator", climberMethods.atBottom());
    SmartDashboard.putString("Elevator State", this.getElevatorState().toString());
    SmartDashboard.putNumber("Motor Output[-1, 1]", climberMethods.getMotorOutputPercent());
    SmartDashboard.putBoolean("Simulating?", !ClimberInterface.isReal());
  }
}
