// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  public static enum IntakeState {
    DISABLED, DEPLOYING, DEPLOYED, STOWING, STOWED, INTAKING
  }

  private IntakeState state;

  /** Creates a new Intake. */
  public Intake() {
    if(Robot.isReal()) {
      Hardware.Intake.pivot = new TalonFX(Constants.Intake.PIVOT_ID);
      Hardware.Intake.roller = new TalonFX(Constants.Intake.ROLLER_ID);
      Hardware.Intake.funnel = new TalonFX(Constants.Intake.FUNNEL_ID);

      Hardware.Intake.pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    } else {
      Hardware.Intake.pivot = new WPI_TalonSRX(Constants.Intake.PIVOT_ID);
      Hardware.Intake.roller = new WPI_TalonSRX(Constants.Intake.ROLLER_ID);
      Hardware.Intake.funnel = new WPI_TalonSRX(Constants.Intake.FUNNEL_ID);

      Hardware.Intake.pivotSim = ((WPI_TalonSRX)Hardware.Intake.pivot).getSimCollection();
    }

    Hardware.Intake.pivot.configFactoryDefault();
    Hardware.Intake.roller.configFactoryDefault();
    Hardware.Intake.funnel.configFactoryDefault();

    Hardware.Intake.pivot.config_kP(0, Constants.Intake.kP);
    Hardware.Intake.pivot.config_kI(0, Constants.Intake.kI);
    Hardware.Intake.pivot.config_kD(0, Constants.Intake.kD);

    setState(IntakeState.DISABLED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

}
