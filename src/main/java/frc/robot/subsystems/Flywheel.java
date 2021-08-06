/**
 * Flywheel.java
 * @version 1.0
 * @since 4/17/21
 * Flywheel Subsystem
 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.utils.RollingAverage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase{
    // all flywheel states
    /**
     * Enum FlywheelState that represents various states of the flywheel -- 
     * OFF: off (not spinning);
     * SPINNINGUP: accelerating/speeding up to target velocity;
     * ATSPEED: at target velocity
     */
    public enum FlywheelState {
        OFF, SPINNINGUP, ATSPEED;
    }
    // Flywheel Hardware
    private BaseTalon flywheelLeader;
    private BaseTalon flywheelFollower;

    //Flywheel attributes
    private FlywheelState currState;
    private double targetRPM = 0;
    private RollingAverage flywheelRPM;
    
    /**
     * Make new Flywheel object/subsystem
     */
    public Flywheel() {
        // Checks if robot is real
        if (RobotBase.isReal()) {
            flywheelLeader = new TalonFX(11);
            flywheelFollower = new TalonFX(12);

            flywheelLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            flywheelFollower.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        }

        flywheelLeader.configFactoryDefault();
        flywheelFollower.configFactoryDefault();

        flywheelLeader.configVoltageCompSaturation(Constants.kVoltageCompensation);
        flywheelFollower.configVoltageCompSaturation(Constants.kVoltageCompensation);

        flywheelLeader.enableVoltageCompensation(true);
        flywheelFollower.enableVoltageCompensation(true);

        flywheelLeader.setInverted(false);
        flywheelFollower.setInverted(false);

        flywheelFollower.follow(flywheelLeader);

        // Sets up PIDF
        flywheelLeader.config_kP(Constants.kPIDIdx, Constants.Flywheel.P);
        flywheelLeader.config_kI(Constants.kPIDIdx, Constants.Flywheel.I);
        flywheelLeader.config_kD(Constants.kPIDIdx, Constants.Flywheel.D);
        flywheelLeader.config_kF(Constants.kPIDIdx, Constants.Flywheel.F);

        targetRPM = 0;
        currState = FlywheelState.OFF;
        flywheelRPM = new RollingAverage(Constants.Flywheel.RPM_STORED);
    }

    /**
     * Set motor target RPM
     * @param desiredVelocity -- desired RPM
     */
    public void setTargetRPM(double desiredVelocity) {
       
        targetRPM = Math.min(6500, desiredVelocity);
        
        if (desiredVelocity == 0)
            setFlywheelState(FlywheelState.OFF);
        else{
            
            setFlywheelState(FlywheelState.SPINNINGUP);
        }
    } 

    /**
     * Updates Smart Dasboard log for Flywheel values
     */
    public void log() {
        SmartDashboard.putNumber("Flywheel RPM", getWheelRPM());
        SmartDashboard.putNumber("Flywheel Needed RPM", getRPM());
        SmartDashboard.putString("Flywheel State", currState.toString());
    }

    /**
     * Get RPM of motor
     * 
     * @return RPM
     */
    public double getWheelRPM() {
        return flywheelLeader.getSelectedSensorVelocity() * 600 / Constants.Flywheel.TICKS_PER_REVOLUTION
                / Constants.Flywheel.GEAR_RATIO;
    }

    /**
     * Set flywheel state 
     * @param newState OFF, ATSPEED
     */
    public void setFlywheelState(FlywheelState newState) {
        this.currState = newState;

        if (newState == FlywheelState.OFF) {
            targetRPM = 0;
        }
        else if (newState == FlywheelState.ATSPEED) {
            targetRPM = getRPM();
        }
    }

    /**
     * Get Flywheel State
     * OFF, SPINNINGUP, ATSPEED
     */
    public FlywheelState getFlywheelState() {
        return currState;
    }

    /**
     * Periodic method
     */
    public void periodic() {
        // updates Rolling Average
        flywheelRPM.updateValue(getWheelRPM());
        log();

       
        // Controls states
        switch (this.currState) {
        case OFF:
            flywheelLeader.set(ControlMode.PercentOutput, 0);
            // flywheelLeader.set(ControlMode.Position, 0);
            break;
        case SPINNINGUP:
            flywheelLeader.set(ControlMode.Velocity, rpmToTicks(targetRPM));
            // Checks if flywheel velocity is within acceptable error
            if (allWithinError(targetRPM, Constants.Flywheel.ACCEPTABLE_ERROR)) {
                setFlywheelState(FlywheelState.ATSPEED);
            }
            break;
        case ATSPEED:
            break;
        }
    }
    
    /**
     * Converts RPM to ticks per 100 ms
     * @param rpm   the rpm to convert
     * @return      the speed in ticks per 100 ms
     */
    private double rpmToTicks(double rpm) {
        return rpm / 600 * Constants.Flywheel.TICKS_PER_REVOLUTION * Constants.Flywheel.GEAR_RATIO;
    }

    /**
     * Function to get necessary RPM from Hardware.limelight distance
     * @return necessary RPM 
     */
    public double getRPM() {
        //finds distance from the target in inches
        double distance_in = (Hardware.limelight.getDistanceFromTarget());

        //mult height and dist by constant and add constant rpm
        return 3.5 * distance_in + 4700;

    }
    /**
     * @param target -- the target RPM
     * @param acceptableError -- the acceptable +- error range
     * @return boolean whether the RPM is within the acceptable error or not
     */
    private boolean allWithinError(double target, double acceptableError) {
        return Math.abs(flywheelRPM.getAverage() - target) <= acceptableError;
    }
}