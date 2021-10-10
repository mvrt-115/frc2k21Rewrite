package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Limelight;

/**
 * Subystem having methods responsible for any actions relating to the
 * drivetrain on the robot
 */
public class Drivetrain extends SubsystemBase {

  /// implement current limit

  // real hardware
  private WPI_TalonFX leftFront; // motor on drivetrain located on the left side in the front
  private WPI_TalonFX rightFront; // motor on drivetrain located on the right side in the front
  private WPI_TalonFX leftBack; // motor on drivetrain located on the left side in the back
  private WPI_TalonFX rightBack; // motor on drivetrain located on the right side in the back
  // keeps track of robot location
  private DifferentialDriveOdometry odometry;

  // returns angular and linear velocity to follow path
  private DifferentialDriveKinematics kinematics;

  // differential drive class
  private DifferentialDrive drive;
  SpeedControllerGroup rightGroup;
  SpeedControllerGroup leftGroup;

  // constants are from robot characteristics
  private SimpleMotorFeedforward feedforward;

	// private RamseteController ramseteController; // auton
	private TrajectoryConfig trajectoryConfig; // auton
  private TrajectoryConfig trajectoryConfigSlow; // auton
  public static final double kMaxVelocityMetersPerSecond = 1.5;
  public static final double kMaxAccelerationMetersPerSecondSq = 2;
  public double integralAcc;


  // PID used by ramsete command
  private PIDController rightPIDController;
  private PIDController leftPIDController;

  // the position of the robot at a certain point in time
  private Pose2d pose;

  // private double lastTime; // tracks time when method was last run for alignment
  // private double totalHorizontalAngleError; // tracks the total error to calculate I in PID of alignment
  // private double lastHorizontalAngleError; // tracks last error to calculate D in PID of alignment

  private SupplyCurrentLimitConfiguration currentLimit;
  private Limelight limelight;
  private AHRS gyro;

  /**
   * Initializes all fields based on whether it is a simulation or the code has
   * been deployed to a robot, and resets both the odometry and gyroscope
   */
  public Drivetrain(Limelight limelight) {
    // initializes if real (plugged into a robot)
    // if (RobotBase.isReal()) {
      leftFront = new WPI_TalonFX(46);
      leftBack = new WPI_TalonFX(9);
      rightFront = new WPI_TalonFX(48);
      rightBack = new WPI_TalonFX(6);
      // leftFront = new TalonFX(46);
      // leftBack = new TalonFX(9);
      // rightFront = new TalonFX(48);
      // rightBack = new TalonFX(6);

      // configures to factory default in case of previous changes
      leftFront.configFactoryDefault();
      leftBack.configFactoryDefault();
      rightFront.configFactoryDefault();
      rightBack.configFactoryDefault();

      // true: enabled, 40: 40 amp current limit, 50: If current crosses 50 amp
      // threshold trigger current limiting, 3.8: if the current exceeds threshold for
      // 3.8 seconds trigger current limiting
      currentLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 3.8);

      leftFront.configSupplyCurrentLimit(currentLimit, Constants.TIMEOUT_MS);
      rightFront.configSupplyCurrentLimit(currentLimit, Constants.TIMEOUT_MS);
      leftBack.configSupplyCurrentLimit(currentLimit, Constants.TIMEOUT_MS);
      rightBack.configSupplyCurrentLimit(currentLimit, Constants.TIMEOUT_MS);

      leftFront.configOpenloopRamp(0.4);
      rightFront.configOpenloopRamp(0.4);
      leftBack.configOpenloopRamp(0.4);
      rightBack.configOpenloopRamp(0.4);

      leftFront.setInverted(false);
      leftBack.setInverted(false);
      rightFront.setInverted(true);
      rightBack.setInverted(true);

      ((TalonFX) leftFront).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PID_IDX,
          Constants.TIMEOUT_MS);
      ((TalonFX) rightFront).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PID_IDX,
          Constants.TIMEOUT_MS);
      ((TalonFX) leftBack).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PID_IDX,
          Constants.TIMEOUT_MS);
      ((TalonFX) rightBack).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PID_IDX,
          Constants.TIMEOUT_MS);
    // }
    gyro = new AHRS(SPI.Port.kMXP);

    leftGroup = new SpeedControllerGroup(leftFront, leftBack);
    rightGroup = new SpeedControllerGroup(rightFront, rightBack);
    
    drive = new DifferentialDrive(leftGroup, rightGroup);
    odometry = new DifferentialDriveOdometry(getGyroRotation2d());

    kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.kTrackWidthMeters);

    // m/s to voltage
    feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.kDriveS, Constants.Drivetrain.kDriveV,
        Constants.Drivetrain.kDriveA);

    rightPIDController = new PIDController(Constants.Drivetrain.kDriveP, Constants.Drivetrain.kDriveI,
        Constants.Drivetrain.kDriveD);
    leftPIDController = new PIDController(Constants.Drivetrain.kDriveP, Constants.Drivetrain.kDriveI,
        Constants.Drivetrain.kDriveD);

    leftFront.configVoltageCompSaturation(Constants.MAX_VOLTAGE);
    rightFront.configVoltageCompSaturation(Constants.MAX_VOLTAGE);
    leftBack.configVoltageCompSaturation(Constants.MAX_VOLTAGE);
    rightBack.configVoltageCompSaturation(Constants.MAX_VOLTAGE);

    leftFront.enableVoltageCompensation(true);
    rightFront.enableVoltageCompensation(true);
    leftBack.enableVoltageCompensation(true);
    rightBack.enableVoltageCompensation(true);

    pose = new Pose2d();
    

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
    this.limelight = limelight;

    trajectoryConfig = new TrajectoryConfig(kMaxVelocityMetersPerSecond,
				kMaxAccelerationMetersPerSecondSq);
		trajectoryConfig.setReversed(false);

		trajectoryConfigSlow = new TrajectoryConfig(.5, .75);
		trajectoryConfigSlow.setReversed(false);

		// ramseteController = new RamseteController();
		integralAcc = 0;


    resetGyro();
    resetEncoderValues();
    resetOdometry();
  }

  /**
   * Generates a ramsete command given a trajectory for auton
   * @param trajectory  the trajectory to follow
   * @return            the ramsete command to follow the trajectory
   */
  public Command getRamseteCommand(Trajectory trajectory) {
    RamseteCommand command = new RamseteCommand(
      trajectory,
      this::getPose,
      new RamseteController(2.0, 7.0),
      getFeedForward(),
      getKinematics(),
      this::getSpeeds,
      getLeftPIDController(),
      getRightPIDController(),
      this::setOutputVoltage
    );

    return command;
  }

  /**
   * Method for regular joystick driving during teleop through implementation of
   * the `syIshDrive, and then applies the speed to the motors
   * 
   * @param throttle  (double) power towards forward and backward movement
   * @param wheel     (double) power towards turning movement
   * @param quickTurn (boolean) status of quickTurnButton
   */
  public void cheesyIshDrive(double throttle, double wheel, boolean quickTurn) {
    throttle = handleDeadband(throttle, Constants.Drivetrain.kThrottleDeadband);
    wheel = handleDeadband(wheel, Constants.Drivetrain.kWheelDeadband);

    double left = 0, right = 0;

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    Twist2d motion = new Twist2d(throttle, 0, wheel);
    if (Math.abs(motion.dtheta) < 1E-9) {
      left = motion.dx;
      right = motion.dx;
    } else {
      double delta_v = Constants.Drivetrain.kTrackWidthInches * motion.dtheta
          / (2 * Constants.Drivetrain.kTrackScrubFactor);
      left = motion.dx + delta_v;
      right = motion.dx - delta_v;
    }

    double scaling_factor = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));

    setDrivetrainMotorSpeed(left / scaling_factor, right / scaling_factor);
  }

  /**
   * Calculates the new input by the joystick after taking into account deadband
   * 
   * @param input         raw input by the joystick
   * @param inputDeadband deadband for the value sent in
   * @return finalInput input by the joystick after calculating deadband
   */
  private double handleDeadband(double input, double inputDeadband) {
    double finalInput = 0;

    if (Math.abs(input) < inputDeadband)
      finalInput = 0;
    else
      finalInput = calculateDeadband(input, inputDeadband);

    return finalInput;
  }

  /**
   * Takes given error and calculates motor output through a PID system for
   * alignment, and then applies it to the motors
   */
  public double alignToTarget() {
    double error = getHorizontalAngleError();
    double distance = limelight.getDistanceFromTarget();
    double kFF = 0.04;  //0.033;
		double kP = .0055;
    double kDist = 0.00005;
		double output;
		integralAcc += error;

		if (Math.abs(error) > 4) {			// .5
			output = error * kP - Math.copySign(kFF, error);
		} else {
			output = error * kP;
		}
    output+=Math.copySign(distance * kDist, error);
    // double horizontalAngleError = getHorizontalAngleError();
    // // find change in error / change in time
    // double dt = Timer.getFPGATimestamp() - lastTime;
    // double de = horizontalAngleError - lastHorizontalAngleError;
    // double slope = de / dt;

    // // if the error is within the range to use integral, add the I term
    // if (Math.abs(horizontalAngleError) < Constants.Drivetrain.kIntegralRange)
    //   totalHorizontalAngleError += dt * horizontalAngleError;

    // // calculate turn speed with PID
    // double turnSpeed = Constants.Drivetrain.kAlignP * horizontalAngleError
    //     + Constants.Drivetrain.kAlignI * totalHorizontalAngleError + Constants.Drivetrain.kAlignD * slope;

    // // if error is greater than constant value, add FF term
    // if (horizontalAngleError > 0.9) {
    //   turnSpeed += Math.copySign(Constants.Drivetrain.kAlignff, horizontalAngleError);
    // }

    // run motors
    setDrivetrainMotorSpeed(output, -output);

    // // change previous angle to current angle
    // lastHorizontalAngleError = horizontalAngleError;
    // // change previous time to current time
    // lastTime = Timer.getFPGATimestamp();

    return output;
  }

  @Override
  /**
   * Periodically updates the odometry, current pose, and the logs
   */
  public void periodic() {
    // This method will be called once per scheduler run

    pose = odometry.update(getGyroRotation2d(), getDistance(leftFront, leftBack), getDistance(rightFront, rightBack));

    log();
  }

  /**
   * Logs all important information on the drivetrain on SmartDashboard to view
   */
  public void log() {
    SmartDashboard.putNumber("Left Encoder", leftFront.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder", rightFront.getSelectedSensorPosition());
    SmartDashboard.putNumber("Current", leftFront.getStatorCurrent() * 4);
    SmartDashboard.putNumber("Curr X Position", pose.getTranslation().getX());
    SmartDashboard.putNumber("Curr Y Position", pose.getTranslation().getY());
    SmartDashboard.putNumber("Horizontal Error", getHorizontalAngleError());
    SmartDashboard.putNumber("NavX", gyro.getAngle());
    SmartDashboard.putNumber("Left Output", leftBack.getMotorOutputPercent());
    SmartDashboard.putNumber("right Output", rightBack.getMotorOutputPercent());
  }

  // ***********************************OUTPUT-SETTING
  // METHODS***********************************//

  /**
   * Sets drivetrain motors to a given left and right percent outunt
   */
  public void setDrivetrainMotorSpeed(double left, double right) {
		leftFront.set(ControlMode.PercentOutput, left);
		leftBack.set(ControlMode.PercentOutput, left);
		rightBack.set(ControlMode.PercentOutput, right);
		rightFront.set(ControlMode.PercentOutput, right);

	}

  /**
   * Sets motor percent output based on voltage to each side of the drivetrain
   * 
   * @param rightVoltage voltage to right side of drivetrain
   * @param leftVoltage  voltage to left side of drivetrain
   */
  // implement voltage compensation
  public void setOutputVoltage(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("volts", leftVolts);
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolts);
    // setDrivetrainMotorSpeed(leftVolts / 100, rightVolts / 100);
    drive.feed();
  }

  /**
   * Sets drivetrain motors to 0% output
   */
  public void stopDrivetrainMotors() {
    setDrivetrainMotorSpeed(0, 0);
  }

  // ***************************************RESET METHODS***************************************//

  /**
   * Sets gyroscope readings to zero
   */
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Resets odometry to zero
   */
  public void resetOdometry() {
    odometry.update(new Rotation2d(), 0, 0);
  }

  /**
   * Resets all encoder values on TalonFX motors
   */
  public void resetEncoderValues() {
    leftFront.setSelectedSensorPosition(0);
    leftBack.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
  }

  // ******************************************GETTERS*****************************************//

  /**
   * Gets the current reading of the gyroscope in Rotation2d
   * 
   * @return Rotation2d of current rotation
   */
  public Rotation2d getGyroRotation2d() {
    return gyro.getRotation2d();
    // return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }
  /**
   * gets the rate at which the gyro is turning
   * @return - turn rate of robot in degrees per second
   */
  public double getRate() {
    return -gyro.getRate();
  }

  /**
   * Gets the angular error for alignment from the limelight
   * 
   * @return angular error (double)
   */
  public double getHorizontalAngleError() {
    return limelight.getTX();
  }

  /**
   * Gets the distance moved by the robot of the motors passed to the method
   * through conversion Precondition: motor1 and motor 2 MUST be on the SAME side
   * of the drivetrain (e.g. leftBack and leftFront)
   * 
   * @param motor1 first motor passed to calculate distance travelled
   * @param motor2 second motor passed to calculate distance travelled
   * @return distance in meters traveled by one side of the drivetrain
   */
  public double getDistance(BaseTalon motor1, BaseTalon motor2) {
    double motorTicks = (motor1.getSelectedSensorPosition() + motor2.getSelectedSensorPosition()) / 2.0;
    double metersTravelled = ticksToMeters(motorTicks);
    return metersTravelled;
  }

  /**
   * Gets the differential drive's speeds
   * 
   * @return DifferentialDriveWheelSpeeds
   */
  public DifferentialDriveWheelSpeeds getSpeeds() {
    double leftMetersPerSecond = RPMToMetersPerSecond(leftFront.getSelectedSensorVelocity());
    double rightMetersPerSecond = RPMToMetersPerSecond(rightFront.getSelectedSensorVelocity());

    return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
  }

  /**
   * Gets the differential drive's kinematics
   * 
   * @return kinematics, a DifferentialDriveKinematics
   */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Gets FeedForward of the motors
   * 
   * @return feedFordward, a SimpleMotorFeedforward
   */
  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  /**
   * Gets the right PID Controller
   * 
   * @return rightPIDController, a PIDController
   */
  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  /**
   * Gets the left PID Controller
   * 
   * @return leftPIDController, a PIDController
   */
  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  /**
   * Gets the current Pose
   * 
   * @return pose, a Pose
   */
  public Pose2d getPose() {
    return pose;
  }

  // ***************************************MATH
  // METHODS**************************************//

  /**
   * Calculates deadband throught an equation that allows low values to be reached
   * even after the deadband is applied.
   * 
   * @param input         original input before deadband
   * @param inputDeadband deadband being applied to the input
   * @return valAfterDeadband new input value after deadband
   */
  private double calculateDeadband(double input, double inputDeadband) {
    double valAfterDeadband = (input - inputDeadband * Math.abs(input) / input) / (1 - inputDeadband);
    // valAfterDeadband = (1 / (1 - inputDeadband)) * (input + (Math.signum(-input)
    // * inputDeadband));
    return valAfterDeadband;
  }

  /**
   * Converts motor rotations per minute to meters per second
   * 
   * @param ticksPer100ms speed in ticks per 100 milliseconds
   * @return metersPerSecond speed in meters per second
   */
  private double RPMToMetersPerSecond(double ticksPer100ms) {
    // conversion: start with ticks per 100 ms -> multiply by 10 to get ticks per
    // second ->
    // divide by ticks per rotation to get rotations per second of gear -> divide by
    // gear
    // ratio to get rotations per second of wheel -> multiply by wheel circumference
    // to get final: meters per second of wheel
    double metersPerSecond = ticksPer100ms / Constants.Drivetrain.kDriveGearRatio
        / Constants.Drivetrain.kFalconTicksPerRotation * Constants.Drivetrain.kWheelCircumferenceMeters * 10 * Math.PI;
    return metersPerSecond;
  }

  /**
   * Converts motor ticks into meters
   * 
   * @param ticks number of ticks to convert
   * @return meters distance in meters
   */
  private double ticksToMeters(double ticks) {
    // conversion: start with ticks -> divide by ticks per rotation to get rotations
    // of the gear ->
    // divide by gear ratio to get rotations of wheel -> multiply by wheel
    // circumference
    // to get final units: meters
    double meters = ticks / Constants.Drivetrain.kDriveGearRatio / Constants.Drivetrain.kFalconTicksPerRotation
        * Constants.Drivetrain.kWheelCircumferenceMeters;
    return meters;
  }

  public void invertPathDirection(boolean reversed){
		trajectoryConfig.setReversed(reversed);
		trajectoryConfigSlow.setReversed(reversed);
  }
  
  public TrajectoryConfig getTrajectoryConfig() {
		return trajectoryConfig;
	}
  public TrajectoryConfig getTrajectoryConfigSlow() {
		return trajectoryConfigSlow;
	}
}