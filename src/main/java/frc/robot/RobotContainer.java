/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static final Hopper hopper = new Hopper();
  private final Intake intake = new Intake();

  TalonFX flywheelMaster = new TalonFX(11);
  TalonFX flywheelFollower = new TalonFX(12);

  public static final Flywheel flywheel = new Flywheel();
  public static final Drivetrain drivetrain = new Drivetrain(); 
  

 /** Main joystick */
 public static Joystick joystick = new Joystick(0);

  public static JoystickButton quickTurnButton = new JoystickButton(joystick,4);

 /** The container for the robot. Contains subsystems, OI devices, and commands. */
 public RobotContainer() {
  //  joystick2 = new Joystick(1);
   //  flywheelMaster.set(ControlMode.PercentOutput, 0.5);
   //  flywheelFollower.set(ControlMode.PercentOutput, 0.5);

   // Configure the button bindings
   configureButtonBindings();
 }

 /**
  * Use this method to define your button->command mappings. Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
  * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
  * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  */
 private void configureButtonBindings() {
   // new JoystickButton(joystick, 1).whenPressed(new ExampleCommand(m_exampleSubsystem, true));

   drivetrain.setDefaultCommand(new JoystickDrive());


  // run intake button
   JoystickButton runIntakeButton =  new JoystickButton( joystick, 1 );
   runIntakeButton.whenPressed( new RunIntake(intake, true ) ).whenReleased(new RunIntake(intake, false));

   // hopper button
      new JoystickButton(joystick,2)
      .whenPressed(new HopperManual(hopper, 0.35, 0.35))
      .whenReleased(new HopperManual(hopper, 0, 0));

   // backwards hopper button
   new JoystickButton(joystick,3)
      .whenPressed(new HopperManual(hopper, -0.35, -0.35))
      .whenReleased(new HopperManual(hopper, 0, 0));

    // shoot flywheel with limelight
    new JoystickButton(joystick,4)
      .whenPressed(new SmartShoot(flywheel))
      .whenReleased(new SetFlywheelRPM(flywheel, 0));;

    // shoot flywheel at set rpm
    new JoystickButton(joystick,7)
      .whenPressed(new SetFlywheelRPM(flywheel, 6000))
      .whenReleased(new SetFlywheelRPM(flywheel, 0));

    // auto hpper
    new JoystickButton(joystick, 6).whenPressed(new HopperAutomatic(hopper)).whenReleased(new HopperManual(hopper, 0, 0));
    // Align robot to target
    new JoystickButton(joystick, 8).whenPressed(new AutoAlign()).whenReleased(new StopDrivetrain());

 }

 /**
   * Gets the throttle from the trigger on the # axis, or __ side trigger
   * @return double from -1 to 1 representing power going back or forth
   */
  public static double getThrottle()
  {
    return -joystick.getRawAxis(5);
  }
  
  /**
   * Gets the wheel from the trigger on the # axis, or __ side trigger
   * @return double from -1 to 1 representing power towards turning
   */
  public static double getWheel()
  {
    return joystick.getRawAxis(0);
  }

  /**
   * Gets the status of the quickTurnButton
   * @return true if button for quickTurn has been pressed, false otherwise
   */
  public static boolean getQuickTurn()
  {
    return quickTurnButton.get();
  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
