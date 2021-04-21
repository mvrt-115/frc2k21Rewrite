/**
 * ClimberMethods.java
 * @version 1.0
 * @since 4/16/2021
 * Provides implementation for the methods defined by ClimberInterface for a simulation
 */

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.robot.Constants;

//Two classes for simulation and real
public class SimulatedClimber implements ClimberInterface 
{
    private double lastTime, lastError, errorSum;               //used for simulation PID
    
    private WPI_TalonSRX elevatorMaster;                        //motor controller
    private ElevatorSim elevatorSim;                            //simulating an elevator
    private Encoder elevatorEncoder;                            //encoder to get the position
    private EncoderSim elevatorEncoderSim;                      //simulating an encoder
    private DCMotor elevatorMotor;                              //DC Motor
    private DigitalInputWrapper elevatorBottomLimitSwitch;      //simulating the elevator bottom limit switch

    public SimulatedClimber(int motorID, int elevatorBottomLimitSwitchID)
    {
        lastTime = lastError = errorSum = 0;

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

        elevatorMaster.configFactoryDefault();
        elevatorEncoder.reset();
        elevatorEncoder.setDistancePerPulse(Constants.Climber.DISTANCE_PER_PULSE);
    }

    /**
     * @see ClimberInterface.java
     */
    public double getDistanceTicks()
    {
        return ClimberInterface.metersToTicks(elevatorSim.getPositionMeters());
    }

    /**
     * @see ClimberInterface.java
     */
    public double getDistance()
    {
        return elevatorEncoder.getDistance();
    }

    /**
     * @see ClimberInterface.java
     */
    public void setPosition(double position)
    {
        double error = ClimberInterface.ticksToMeters(position) - getDistance();
        double currTime = Timer.getFPGATimestamp();
        double dt = currTime - lastTime;

        errorSum += error;
        double output = Constants.Climber.kElevatorP * error + Constants.Climber.kElevatorI * errorSum + Constants.Climber.kElevatorD * (error - lastError)/dt;

        elevatorMaster.set(output);
        lastError = error;
        lastTime = currTime;
    }

    /**
     * @see ClimberInterface.java
     */
    public void simulate(Servo elevatorServo)
    {
        if(elevatorSim.getPositionMeters() == 0)
           elevatorBottomLimitSwitch.set(true); 
        else 
            elevatorBottomLimitSwitch.set(false);
        if(elevatorServo.getAngle() != Constants.Climber.kServoRatchet)
        {
            elevatorSim.setInput(elevatorMaster.getMotorOutputVoltage());
            elevatorSim.update(0.020);
            elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
            RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
        }
    }
    
    /**
     * @see ClimberInterface.java
     */
    public boolean atBottom()
    {
        return elevatorBottomLimitSwitch.get();
    }

    /**
     * @see ClimberInterface.java
     */
    public void climb()
    {
        this.setPosition(Constants.Climber.kClimbHeight);
    }

    /**
     * @see ClimberInterface.java
     */
    public void zero()
    {
        this.setPosition(Constants.Climber.kElevatorZero);
    }

    /**
     * @see ClimberInterface.java
     */
    public void stop()
    {
        elevatorMaster.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * @see ClimberInterface.java
     */
    public double getMotorOutputPercent()
    {
        return elevatorMaster.getMotorOutputPercent();
    }

    /**
     * @see ClimberInterface.java
     */
    public void setMotorOutputPercent(double percent)
    {
        elevatorMaster.set(ControlMode.PercentOutput, percent);
    }
}
