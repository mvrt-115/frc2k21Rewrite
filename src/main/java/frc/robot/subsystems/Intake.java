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
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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

  private boolean limitSwitchInitial;
  private double feedForward;

  /** Pivot Talon */
  private BaseTalon pivot;
  
   /** Roller Talon */
  private BaseTalon roller;
  
   /** Funnel Talon */
  private BaseTalon funnel;
  
  private DigitalInput limitSwitchBottom;
  
  private TalonSRXSimCollection pivotControllerSim;

  private DifferentialDrivetrainSim pivotSim;
  
  /** Creates a new Intake. */

  public Intake() {
    // initialize differently based on if it is a real or simulated robot
    if(Robot.isReal()) {
      pivot = new TalonSRX(Constants.Intake.PIVOT_ID);
      roller = new TalonSRX(Constants.Intake.ROLLER_ID);
      funnel = new TalonSRX(Constants.Intake.FUNNEL_ID);

      pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    } else {
      pivot = new WPI_TalonSRX(Constants.Intake.PIVOT_ID);
      roller = new WPI_TalonSRX(Constants.Intake.ROLLER_ID);
      funnel = new WPI_TalonSRX(Constants.Intake.FUNNEL_ID);

      pivotControllerSim = ((WPI_TalonSRX)pivot).getSimCollection();

      pivotSim = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(1), 
        Constants.Intake.PIVOT_GEAR_RATIO, 
        Constants.Intake.ROTATIONAL_INERTIA, 
        Constants.Intake.ROBOT_MASS, 
        Constants.Intake.WHEEL_RADIUS, 
        Constants.Intake.TRACK_WIDTH, 
        null
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

    limitSwitchInitial = limitSwitchBottom.get();

    setState(IntakeState.DISABLED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    feedForward = Constants.Intake.FF * Math.sin(Math.toRadians(getPivotTicks()));
    switch( state )
    {
      case INTAKING:
        intake();
        break;

      case STOWING:
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
    pivotSim.setInputs(pivot.getMotorOutputVoltage(), pivot.getMotorOutputVoltage());

    // sets the simulated position of the pivot
    pivotControllerSim.setQuadratureRawPosition(metersToTicks(pivotSim.getLeftPositionMeters()));

    //Does some funky math to determine the current voltage of the battery based on the current pull of the drivetrain motors
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));
  }

  /**
   * Converts meters into ticks
   * @param meters  the amount of meters to be converted
   * @return        the amount of ticks in the given amount of meters
   */
  public int metersToTicks(double meters) {
    double wheelCircumference_Meters = 2 * Math.PI * Constants.Intake.WHEEL_RADIUS;
    double rotations = meters / wheelCircumference_Meters;
    double motorRotations = rotations * Constants.Intake.PIVOT_GEAR_RATIO;
    int ticks = (int)(motorRotations * Constants.Intake.PIVOT_TICKS_PER_REVOLUTION);

    return ticks;
  }

  /**
   * Deploys the intake, if the intake has reached the bottom the intake is deployed and will begin to intake
   */
  public void deploy() {
    if(limitSwitchBottom.get() != limitSwitchInitial 
        || getPivotTicks() >= degreesToTicks(Constants.Intake.PIVOT_DEPLOYED_DEGREES)) {
      stopPivot();
      setState(IntakeState.INTAKING);
      intake();
    } else {
      pivot(Constants.Intake.PIVOT_DEPLOYED_DEGREES);
    }    
  }

  /**
   * Stows the intake until the intake has reached the top
   */
  public void stow() {
    if(getPivotTicks() <= Constants.Intake.PIVOT_STOWED_DEGREES) {
      stopPivot();
      setState(IntakeState.DISABLED);
    } else {
      pivot(Constants.Intake.PIVOT_STOWED_DEGREES);
    }    
  }

  /**
   * Pivots the intake to a desired target angle
   * @param target_degrees  The target angle in degrees
   */
  public void pivot(double target_degrees)
  {
    pivot.set(
      ControlMode.Position, 
      degreesToTicks(target_degrees), 
      DemandType.ArbitraryFeedForward, 
      feedForward
    );
  }

  /**
   * Runs the intake by running it at a constant speed
   */
  public void intake()
  {
    funnel.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_SPEED);
    roller.set(ControlMode.PercentOutput, Constants.Intake.INTAKE_SPEED);
  }
  
  /**
   * Stops the pivot motor
   */
  public void stopPivot()
  {
    pivot.set(ControlMode.PercentOutput, 0);
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
   * Converts ticks to degrees
   * @param ticks a position in ticks
   * @return      the angle in degrees of the position
   */
  public double ticksToDegrees(double ticks) {
    return 360 * ticks / (Constants.Intake.PIVOT_TICKS_PER_REVOLUTION * Constants.Intake.PIVOT_GEAR_RATIO);
  }

  /**
   * Logs data about the subsystem to smart dashboard
   */
  public void log() {
    SmartDashboard.putString("Intake State", state.toString());
    SmartDashboard.putNumber("Intake Position (degrees)", ticksToDegrees(getPivotTicks()));
    SmartDashboard.putBoolean("Intake Running", getState() == IntakeState.INTAKING);
    SmartDashboard.putNumber("Pivot Output", pivot.getMotorOutputPercent());
  }
}
