/**
 * ClimberMethods.java
 * @version 1.0
 * @since 4/16/2021
 * Provides implementation for the methods defined by ClimberInterface
 */

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

//Two classes for simulation and real
public class RealClimber implements ClimberInterface 
{
    private TalonFX elevatorMaster;
    private DigitalInput elevatorBottomLimitSwitch;
    private SupplyCurrentLimitConfiguration currentConfig;    //limiting the current flow into the elevator

    public RealClimber(int motorID, int elevatorBottomLimitSwitchID) 
    {
        elevatorMaster = new TalonFX(motorID);
        elevatorBottomLimitSwitch = new DigitalInput(elevatorBottomLimitSwitchID);
        elevatorMaster.configFactoryDefault();

        elevatorMaster.setInverted(true);
      
        elevatorMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx, Constants.kTimeoutMs);

        currentConfig = new SupplyCurrentLimitConfiguration(true, 40, 1.3, 30);
        elevatorMaster.configSupplyCurrentLimit(currentConfig);
    }

    /**
     * @see ClimberInterface.java
     */
    public double getDistanceTicks()
    {
        return elevatorMaster.getSelectedSensorPosition();
    }

    /**
     * @see ClimberInterface.java
     */
    public double getDistance()
    {
        return ClimberInterface.ticksToMeters(elevatorMaster.getSelectedSensorPosition());
    }

    /**
     * @see ClimberInterface.java
     */
    public void setPosition(double position)
    {
        elevatorMaster.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, Constants.Climber.kElevatorClimbOutput);
    }

    /**
     * @see ClimberInterface.java
     */
    public void simulate(Servo servo) throws UnsupportedOperationException
    {
        throw new UnsupportedOperationException("Cannot simulate limit switch for a real robot");
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
    public boolean atBottom()
    {
        return elevatorBottomLimitSwitch.get();
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
