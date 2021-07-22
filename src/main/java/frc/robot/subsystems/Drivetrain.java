package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;


/**
 * Subystem having methods responsible for any actions relating to the 
 * drivetrain on the robot
 */
public class Drivetrain extends SubsystemBase {

/// implement current limit


  //real hardware
  private BaseTalon leftFront; // motor on drivetrain located on the left side in the front
  private BaseTalon rightFront; // motor on drivetrain located on the right side in the front
  private BaseTalon leftBack; // motor on drivetrain located on the left side in the back
  private BaseTalon rightBack; // motor on drivetrain located on the right side in the back

  //keeps track of robot location
  private DifferentialDriveOdometry odometry;

  //returns angular and linear velocity to follow path
  private DifferentialDriveKinematics kinematics;

  //constants are from robot characteristics
  private SimpleMotorFeedforward feedforward;

  //PID used by ramsete command
  private PIDController rightPIDController;
  private PIDController leftPIDController;

  //the position of the robot at a certain point in time
  private Pose2d pose;

  private double lastTime; //tracks time when method was last run for alignment
  private double totalHorizontalAngleError; //tracks the total error to calculate I in PID of alignment
  private double lastHorizontalAngleError; //tracks last error to calculate D in PID of alignment

  private boolean align;

  /**
   * Initializes all fields based on whether it is a simulation or the code has been deployed to a robot,
   * and resets both the odometry and gyroscope
   */
  public Drivetrain() 
  {
    //initializes if real (plugged into a robot)
    if(RobotBase.isReal())
    {
      leftFront = new TalonFX(46);
      leftBack = new TalonFX(9);
      rightFront = new TalonFX(48);
      rightBack = new TalonFX(6);

      //configures to factory default in case of previous changes
      leftFront.configFactoryDefault();
      leftBack.configFactoryDefault();
      rightFront.configFactoryDefault();
      rightBack.configFactoryDefault();

      leftFront.configOpenloopRamp(1);
      rightFront.configOpenloopRamp(1);

      //no motor runs inverted
      leftFront.setInverted(false);
      leftBack.setInverted(false);
      rightFront.setInverted(true);
      rightBack.setInverted(true);

      ((TalonFX)leftFront).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
				Constants.kTimeoutMs);
      ((TalonFX)rightFront).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
        Constants.kTimeoutMs);
      ((TalonFX)leftBack).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
        Constants.kTimeoutMs);
      ((TalonFX)rightBack).configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
        Constants.kTimeoutMs);
    }

    align = false;

    leftFront.configVoltageCompSaturation(Constants.Drivetrain.kVoltageCompensation);
    rightFront.configVoltageCompSaturation(Constants.Drivetrain.kVoltageCompensation);
    leftBack.configVoltageCompSaturation(Constants.Drivetrain.kVoltageCompensation);
    rightBack.configVoltageCompSaturation(Constants.Drivetrain.kVoltageCompensation);

    leftFront.enableVoltageCompensation(true);
    rightFront.enableVoltageCompensation(true);
    leftBack.enableVoltageCompensation(true);
    rightBack.enableVoltageCompensation(true);
    
    Hardware.gyro = new AHRS(SPI.Port.kMXP);

    odometry = new DifferentialDriveOdometry(getGyroAngle());

    kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.kTrackWidthMeters);

    //m/s to voltage
    feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.kDriveS, Constants.Drivetrain.kDriveV, 
        Constants.Drivetrain.kDriveA);

    rightPIDController = new PIDController(Constants.Drivetrain.kDriveP, Constants.Drivetrain.kDriveI, 
        Constants.Drivetrain.kDriveD);
    leftPIDController = new PIDController(Constants.Drivetrain.kDriveP, Constants.Drivetrain.kDriveI, 
        Constants.Drivetrain.kDriveD);
    
    // leftBack.follow(leftFront);
    // rightBack.follow(rightFront);


    
    resetGyro();

    resetEncoderValues();
    resetOdometry();
  }

  /**
   * Method for regular joystick driving during teleop through implementation of the
   * `syIshDrive, and then applies the speed to the motors
   * @param throttle      (double) power towards forward and backward movement 
   * @param wheel         (double) power towards turning movement
   * @param quickTurn     (boolean) status of quickTurnButton
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
      double delta_v = Constants.Drivetrain.kTrackWidthInches * motion.dtheta / (2 * 
          Constants.Drivetrain.kTrackScrubFactor);
			left = motion.dx + delta_v;
			right = motion.dx - delta_v;
		}

    double scaling_factor = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));

    setDrivetrainMotorSpeed(left / scaling_factor, right / scaling_factor);
  }

  /**
   * Calculates the new input by the joystick after taking into account deadband
   * @param input               raw input by the joystick
   * @param inputDeadband       deadband for the value sent in
   * @return finalInput         input by the joystick after calculating deadband
   */
  private double handleDeadband(double input, double inputDeadband)
  {
    double finalInput = 0;

    if( Math.abs(input) < inputDeadband)
      finalInput = 0;
    else
      finalInput = calculateDeadband(input, inputDeadband);
    
    return finalInput;
  }

  /**
   * Takes given error and calculates motor output through a PID system for alignment,
   * and then applies it to the motors
   * @param horizontalAngleError  angular error to target
   */
  public double alignToTarget(double horizontalAngleError)
  {
    // find change in error / change in time
    double dt = Timer.getFPGATimestamp() - lastTime;
    double de = horizontalAngleError - lastHorizontalAngleError;
    double slope = de/dt;

    // if the error is within the range to use integral, add the I term
    if(Math.abs(horizontalAngleError) < Constants.Drivetrain.kIntegralRange)
      totalHorizontalAngleError += dt*horizontalAngleError;

    // calculate turn speed with PID
    double turnSpeed = Constants.Drivetrain.kAlignP*horizontalAngleError + 
        Constants.Drivetrain.kAlignI*totalHorizontalAngleError + Constants.Drivetrain.kAlignD*slope ;

    // if error is greater than constant value, add FF term
    if(horizontalAngleError > 1.5){
      turnSpeed += Math.copySign(Constants.Drivetrain.kAlignff, horizontalAngleError);
    }

    // run motors
    setDrivetrainMotorSpeed(turnSpeed, -turnSpeed);

    SmartDashboard.putNumber("Turn Speed", turnSpeed);

    // change previous angle to current angle
    lastHorizontalAngleError = horizontalAngleError;
    // change previous time to current time
    lastTime = Timer.getFPGATimestamp();

    return turnSpeed;
  }
  
  @Override
  /**
   * Periodically updates the odometry, current pose, and the logs
   */
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(getGyroAngle(), getDistance(leftFront, leftBack), getDistance(rightFront, rightBack));
    pose = odometry.getPoseMeters();

    if(align) {
      alignToTarget(getHorizontalAngleError());
    }

    log();
  }

  /**
   * Logs all important information on the drivetrain on SmartDashboard to view
   */
  public void log()
  {
    SmartDashboard.putNumber("Left Encoder", leftFront.getSelectedSensorPosition());
		SmartDashboard.putNumber("Right Encoder", rightFront.getSelectedSensorPosition());
		SmartDashboard.putNumber("Current", leftFront.getStatorCurrent()* 4);
    SmartDashboard.putNumber("Curr X Position", pose.getTranslation().getX());
    SmartDashboard.putNumber("Curr Y Position", pose.getTranslation().getY());
    SmartDashboard.putNumber("Horizontal Error", getHorizontalAngleError());
    SmartDashboard.putBoolean("Aligning", align);
    SmartDashboard.putNumber("NavX", Hardware.gyro.getAngle());
  }


  //***********************************OUTPUT-SETTING METHODS***********************************//

  /**
   * Sets drivetrain motors to a given left and right percent outunt
   */
  public void setDrivetrainMotorSpeed(double leftSpeed, double rightSpeed)
  {
    leftFront.set(ControlMode.PercentOutput, leftSpeed);
    rightFront.set(ControlMode.PercentOutput, rightSpeed);
  }

  /**
   * Sets motor percent output based on voltage to each side of the drivetrain
   * @param rightVoltage      voltage to right side of drivetrain
   * @param leftVoltage       voltage to left side of drivetrain
   */
  //implement voltage compensation
  public void setOutputVoltage(double rightVoltage, double leftVoltage)
  {
    setDrivetrainMotorSpeed(leftVoltage / RobotController.getBatteryVoltage(), rightVoltage / 
        RobotController.getBatteryVoltage());
  }

  /**
   * Sets drivetrain motors to 0% output
   */
  public void stopDrivetrainMotors()
  {
    setDrivetrainMotorSpeed(0, 0);
    align = false;
  }

  public void align() {
    align = true;
  }

  //***************************************RESET METHODS***************************************//

  /**
   * Sets gyroscope readings to zero
   */
  public void resetGyro()
  {
    Hardware.gyro.reset();
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

  //******************************************GETTERS*****************************************//

  /**
   * Gets the current reading of the gyroscope in Rotation2d
   * @return Rotation2d of current rotation
   */
  public Rotation2d getGyroAngle()
  {
    return Rotation2d.fromDegrees(-Hardware.gyro.getAngle());
  }

  /**
   * Gets the angular error for alignment from the limelight
   * @return  angular error (double)
   */
  public double getHorizontalAngleError()
  {
    return Hardware.limelight.getTX();
  }

  /**
   * Gets the distance moved by the robot of the motors passed to the method through conversion
   * Precondition: motor1 and motor 2 MUST be on the SAME side of the drivetrain (e.g. leftBack and leftFront)
   * @param motor1      first motor passed to calculate distance travelled
   * @param motor2      second motor passed to calculate distance travelled
   * @return distance in meters traveled by one side of the drivetrain
   */
  public double getDistance(BaseTalon motor1, BaseTalon motor2)
  {
    double motorTicks = (motor1.getSelectedSensorPosition() 
      + motor2.getSelectedSensorPosition())/2.0;
    double metersTravelled = ticksToMeters(motorTicks);
    return metersTravelled;
  }

  /**
   * Gets the differential drive's speeds
   * @return DifferentialDriveWheelSpeeds
   */
  public DifferentialDriveWheelSpeeds getSpeeds()
  {
    double leftMetersPerSecond = RPMToMetersPerSecond(leftFront.getSelectedSensorVelocity());
    double rightMetersPerSecond = RPMToMetersPerSecond(rightFront.getSelectedSensorVelocity());

    return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
  }

  /**
   * Gets the differential drive's kinematics
   * @return kinematics, a DifferentialDriveKinematics
   */
  public DifferentialDriveKinematics getKinematics()
  {
    return kinematics;
  }

  /**
   * Gets FeedForward of the motors
   * @return feedFordward, a SimpleMotorFeedforward
   */
  public SimpleMotorFeedforward getFeedForward()
  {
    return feedforward;
  }

  /**
   * Gets the right PID Controller
   * @return rightPIDController, a PIDController
   */
  public PIDController getRightPIDController()
  {
    return rightPIDController;
  }

  /**
   * Gets the left PID Controller
   * @return leftPIDController, a PIDController
   */
  public PIDController getLeftPIDController()
  {
    return leftPIDController;
  }

  /**
   * Gets the current Pose
   * @return pose, a Pose
   */
  public Pose2d getPose()
  {
    return pose;
  }

  //***************************************MATH METHODS**************************************//
  
  /**
   * Calculates deadband throught an equation that allows low values to be reached
   * even after the deadband is applied.
   * @param input               original input before deadband
   * @param inputDeadband       deadband being applied to the input
   * @return valAfterDeadband   new input value after deadband
   */
  private double calculateDeadband(double input, double inputDeadband)
  {
    double valAfterDeadband = (input - inputDeadband * Math.abs(input) / input) / (1 - inputDeadband);
    // valAfterDeadband = (1 / (1 - inputDeadband)) * (input + (Math.signum(-input) * inputDeadband));
    return valAfterDeadband;
  }

  /**
   * Converts motor rotations per minute to meters per second
   * @param ticksPer100ms       speed in ticks per 100 milliseconds
   * @return metersPerSecond    speed in meters per second
   */
  private double RPMToMetersPerSecond(double ticksPer100ms)
  {
    //conversion: start with ticks per 100 ms -> multiply by 10 to get ticks per second ->
                  // divide by ticks per rotation to get rotations per second of gear -> divide by gear
                  // ratio to get rotations per second of wheel -> multiply by wheel circumference
                  // to get final: meters per second of wheel
    double metersPerSecond = ticksPer100ms / Constants.Drivetrain.kDriveGearRatio / 
        Constants.Drivetrain.kFalconTicksPerRotation * Constants.Drivetrain.kWheelCircumferenceMeters * 10;
    return metersPerSecond;
  }

  /**
   * Converts motor ticks into meters
   * @param ticks      number of ticks to convert
   * @return meters    distance in meters
   */
  private double ticksToMeters(double ticks)
  {
    //conversion: start with ticks -> divide by ticks per rotation to get rotations of the gear ->
                  // divide by gear ratio to get rotations of wheel -> multiply by wheel circumference
                  // to get final units: meters
    double meters = ticks / Constants.Drivetrain.kDriveGearRatio / 
        Constants.Drivetrain.kFalconTicksPerRotation * Constants.Drivetrain.kWheelCircumferenceMeters;
    return meters;
  }
}