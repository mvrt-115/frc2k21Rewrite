// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  public static enum IntakeState {
    DISABLED, DEPLOYING, STOWING, INTAKING
  }

  /**The state of the intake */
  private IntakeState state;

  private boolean limitSwitchInitial;
  private double feedForward;

  /** Pivot Talon */
  private BaseTalon pivot;
  
   /** Roller Talon */
  private BaseTalon roller;
  
   /** Funnel Talon */
  private BaseTalon funnel;
  
  private DigitalInput limitSwitchBottom;
  
  /** Creates a new Intake. */

  public Intake() {
    // initialize differently based on if it is a real or simulated robot
    if(Robot.isReal()) {
      pivot = new TalonSRX(Constants.Intake.PIVOT_ID);
      roller = new TalonSRX(Constants.Intake.ROLLER_ID);
      funnel = new TalonSRX(Constants.Intake.FUNNEL_ID);

      pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    // reset to factory defaults
    pivot.configFactoryDefault();
    roller.configFactoryDefault();
    funnel.configFactoryDefault();

    roller.setInverted(true);
    funnel.setInverted(false);

    // configure PID constants and feed forward to compensate for gravity
    pivot.config_kP(0, Constants.Intake.P);
    pivot.config_kI(0, Constants.Intake.I);
    pivot.config_kD(0, Constants.Intake.D);

    pivot.enableVoltageCompensation(true);

    pivot.configVoltageCompSaturation(Constants.kVoltageCompensation);
    roller.configVoltageCompSaturation(Constants.kVoltageCompensation);
    funnel.configVoltageCompSaturation(Constants.kVoltageCompensation);

    pivot.enableVoltageCompensation(true);
    roller.enableVoltageCompensation(true);
    funnel.enableVoltageCompensation(true);

    limitSwitchBottom = new DigitalInput(Constants.Intake.LIMIT_SWITCH_ID);

    limitSwitchInitial = limitSwitchBottom.get();

    setState(IntakeState.DISABLED);
    //System.out.println(state);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    feedForward = Constants.Intake.FF * Math.cos(Math.toRadians(getAngle()));

    switch( state )
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
        pivot.setNeutralMode(NeutralMode.Brake);
        break;

      default: 
        break;
    }
    log();
  }

  /**
   * Deploys the intake, if the intake has reached the bottom the intake is deployed and will begin to intake
   */
  public void deploy() {
    if(limitSwitchBottom.get() != limitSwitchInitial 
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
  public void pivot(double target_ticks)
  {
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
  public void intake()
  {
    funnel.set(ControlMode.PercentOutput, Constants.Intake.FUNNEL_SPEED);
    roller.set(ControlMode.PercentOutput, Constants.Intake.ROLLER_SPEED);
  }
  
  /**
   * Stops the pivot motor
   */
  public void stopPivot()
  {
    // pivot.set(ControlMode.PercentOutput, 0);
    pivot.set(ControlMode.PercentOutput, -0.15);
  }

  /**
   * Stops the intake motors
   */
  public void stopIntake()
  {
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
  public double degreesToTicks(double degrees) {
    return degrees / 360 * Constants.Intake.PIVOT_TICKS_PER_REVOLUTION * Constants.Intake.PIVOT_GEAR_RATIO;
  }

  /**
   * Finds the current position of the pivot in ticks
   * @return the current position of the pivot in ticks
   */
  public double getPivotTicks() {
    return pivot.getSelectedSensorPosition();
  }

  /**
   * Logs data about the subsystem to smart dashboard
   */
  public void log() {
    SmartDashboard.putString("Intake State", state.toString());
    SmartDashboard.putNumber("Intake Current Ticks", getPivotTicks());
  }

  /**
   * Finds the current angle of the intake pivot
   * @return  the angle of the pivot in degrees
   */
  public double getAngle() {
    return 90 + (getPivotTicks() / 1000 * 100);
  }
}