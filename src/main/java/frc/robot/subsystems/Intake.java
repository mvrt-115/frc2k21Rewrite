// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  public static enum IntakeState {
    DISABLED, DEPLOYING, STOWING, INTAKING
  }

  /**The state of the intake */
  private IntakeState state;

  private boolean isBreakBeamOpen;
  private double feedForward;

  /** Pivot Talon */
  private BaseTalon pivot;
  
   /** Roller Talon */
  private BaseTalon roller;
  
   /** Funnel Talon */
  private BaseTalon funnel;
  
  private DigitalInput limitSwitchBottom;
  
  private TalonSRXSimCollection pivotControllerSim;

  private SingleJointedArmSim pivotSim;
  
  /** Creates a new Intake. */

  public Intake() {
    // initialize differently based on if it is a real or simulated robot
    if(Robot.isReal()) {
      pivot = new TalonSRX(Constants.Intake.PIVOT_ID);
      roller = new TalonSRX(Constants.Intake.ROLLER_ID);
      funnel = new TalonSRX(Constants.Intake.FUNNEL_ID);

      pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    } else {
      pivot = new WPI_TalonSRX(Constants.Intake.PIVOT_SIM_ID);
      roller = new WPI_TalonSRX(Constants.Intake.ROLLER_SIM_ID);
      funnel = new WPI_TalonSRX(Constants.Intake.FUNNEL_SIM_ID);

      pivotControllerSim = ((WPI_TalonSRX)pivot).getSimCollection();

      // arm simulation
      pivotSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1), 
        Constants.Intake.PIVOT_GEAR_RATIO,
        Constants.Intake.ROTATIONAL_INERTIA,
        Constants.Intake.PIVOT_LENGTH,
        Constants.Intake.PIVOT_MIN_ANGLE,
        Constants.Intake.PIVOT_MAX_ANGLE, 
        Constants.Intake.PIVOT_MASS,
        false
      );
    }

    
    // reset to factory defaults
    pivot.configFactoryDefault();
    roller.configFactoryDefault();
    funnel.configFactoryDefault();

    // configure PID constants and feed forward to compensate for gravity
    pivot.config_kP(0, Constants.Intake.P);
    pivot.config_kI(0, Constants.Intake.I);
    pivot.config_kD(0, Constants.Intake.D);
    
    // voltage compensation
    pivot.enableVoltageCompensation(true);
    roller.enableVoltageCompensation(true);
    funnel.enableVoltageCompensation(true);

    limitSwitchBottom = new DigitalInput(Constants.Intake.LIMIT_SWITCH_ID);

    isBreakBeamOpen = limitSwitchBottom.get();

    feedForward = 0;

    setState(IntakeState.DISABLED);
  }

  @Override
  public void periodic() {
    feedForward = Constants.Intake.FF * Math.cos(Math.toRadians(ticksToDegrees(getPivotTicks())));

    // This method will be called once per scheduler run
    switch(state)
    {
      case INTAKING:
        intake();
        break;

      case STOWING:
        stopIntake();
        stow();
        break;

      case DEPLOYING:
        deploy();
        break;

      case DISABLED:
        stopIntake();
        stopPivot();
        break;

      default: 
        break;
    }
    log();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // set the simulated inputs
    pivotSim.setInputVoltage(pivot.getMotorOutputVoltage());

    // sets the simulated position of the pivot
    pivotControllerSim.setQuadratureRawPosition(degreesToTicks(Math.toDegrees(pivotSim.getAngleRads())));
    pivotControllerSim.setAnalogPosition(degreesToTicks(Math.toDegrees(pivotSim.getAngleRads())));

    // simulate limit switch

    //Does some funky math to determine the current voltage of the battery based on the current pull of the drivetrain motors
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));
    
    pivotSim.update(0.02);
  }

  /**
   * Deploys the intake, if the intake has reached the bottom the intake is deployed and will begin to intake
   */
  public void deploy() {
    if(limitSwitchBottom.get() != isBreakBeamOpen 
        || Math.abs(getPivotTicks() - Constants.Intake.PIVOT_DEPLOYED_TICKS) 
        <= Constants.Intake.MARGIN_OF_ERROR_TICKS) {
      stopPivot();
      setState(IntakeState.INTAKING);
      intake();
    } else {
      pivot(Constants.Intake.PIVOT_DEPLOYED_TICKS);
    }    
  }

  /**
   * Stows the intake until the intake has reached the top
   */
  public void stow() {    
    if(Math.abs(getPivotTicks() - Constants.Intake.PIVOT_STOWED_TICKS) 
        <= Constants.Intake.MARGIN_OF_ERROR_TICKS) {
      stopPivot();
      setState(IntakeState.DISABLED);
    } else {
      pivot(Constants.Intake.PIVOT_STOWED_TICKS);
    }    
  }

  /**
   * Pivots the intake to a desired target angle
   * @param target_ticks  The target angle in ticks
   */
  public void pivot(double target_ticks) {
    if(Robot.isSimulation())
      feedForward = 0;

    pivot.set(
      ControlMode.Position, 
      target_ticks, 
      DemandType.ArbitraryFeedForward, 
      feedForward
    );
  }

  /**
   * Runs the intake by running it at a constant speed
   */
  public void intake() {
    funnel.set(ControlMode.PercentOutput, Constants.Intake.FUNNEL_SPEED);
    roller.set(ControlMode.PercentOutput, Constants.Intake.ROLLER_SPEED);
  }
  
  /**
   * Stops the pivot motor
   */
  public void stopPivot() {
    pivot.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Stops the intake motors
   */
  public void stopIntake() {
    funnel.set(ControlMode.PercentOutput, 0);
    roller.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return IntakeState return the state
   */
  public IntakeState getState() {
    return state;
  }

  /**
   * @param state the state to set
   */
  public void setState(IntakeState state) {
    this.state = state;
  }

  /**
   * Converts degrees to ticks
   * @param degrees an anlge in degrees
   * @return        the amount of ticks in that angle
   */
  public int degreesToTicks(double degrees) {
    return (int)((degrees - 90) * Constants.Intake.PIVOT_MAX_TICKS / 100);
  }

  /**
   * Finds the current position of the pivot in ticks
   * @return the current position of the pivot in ticks
   */
  public double getPivotTicks() {
    return pivot.getSelectedSensorPosition();
  }

  /**
   * Converts ticks to degrees
   * @param ticks the amount of ticks
   * @return      the amount of degrees fround from the given ticks
   */
  public double ticksToDegrees(double ticks) {
    return 90 + (ticks / Constants.Intake.PIVOT_MAX_TICKS * 100);
  }

  /**
   * Logs data about the subsystem to smart dashboard
   */
  public void log() {
    SmartDashboard.putString("Intake State", state.toString());
    SmartDashboard.putBoolean("Intake Running", getState() == IntakeState.INTAKING);
    SmartDashboard.putNumber("Intake Pivot Output", pivot.getMotorOutputPercent());
    SmartDashboard.putNumber("Intake Current Ticks", getPivotTicks());
    SmartDashboard.putNumber("Intake Current Angle", ticksToDegrees(getPivotTicks()));
    SmartDashboard.putNumber("Intake Feed Forward", feedForward);
  }
}
