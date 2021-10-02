/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Limelight;
import frc.robot.utils.RollingAverage;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Hopper hopper = new Hopper();
  public final Intake intake = new Intake();
  public final Limelight limelight = new Limelight();

  public final Flywheel flywheel = new Flywheel(limelight);  
  public final Drivetrain drivetrain = new Drivetrain(limelight);
  private SendableChooser<Command> autonSelector;

  
  /** Main joystick */

   // operator
   // hopper manual
   // elevator stuff, servo and climbing
   // hopper ball count fix
   // shoot
  public Joystick joystick = new Joystick(0);
  public Joystick opJoystick = new Joystick(1);

  // joystick buttons

  public JoystickButton quickturn = new JoystickButton(joystick, 5);
  // public JoystickButton runIntake = new JoystickButton(joystick, 1);
  public JoystickButton autoHopper = new JoystickButton(opJoystick, 6);
  // public JoystickButton autoAlign = new JoystickButton(joystick, 8);
  public JoystickButton autoAlign = new JoystickButton(joystick, 1);
  public JoystickButton hopperDown = new JoystickButton(opJoystick, 1);
  public JoystickButton hopperUp = new JoystickButton(opJoystick, 4);
  public JoystickButton shootToTarget = new JoystickButton(joystick, 6);
  public JoystickButton resetHopper = new JoystickButton(opJoystick, 3);
  // public JoystickButton alignShoot = new JoystickButton(joystick, 8);
  // public JoystickButton shoot = new JoystickButton(joystick, 7);

  public RollingAverage throttle = new RollingAverage(50);
  public RollingAverage wheel = new RollingAverage(15);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  public Intake getIntake() {
    return intake;
  }
  public Hopper getHopper() {
    return hopper;
  }
  public Drivetrain getDrivetrain() {
    return drivetrain;
  }
  public Flywheel getFlywheel() {
    return flywheel;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(joystick, 1).whenPressed(new
    // ExampleCommand(m_exampleSubsystem, true));

    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, this::getThrottle, this::getWheel, this::getQuickTurn));

    // run intake button
    // runIntake.whenPressed(new RunIntake(intake, true)).whenReleased(new RunIntake(intake, false));

    // // hopper button
    autoAlign.whenPressed(new AutoAlign(drivetrain)).whenReleased(new RunDrivetrain(drivetrain, 0));

    // // backwards hopper button
    hopperDown.whenPressed(new HopperManual(hopper, -0.35, -0.35)).whenReleased(new HopperManual(hopper, 0, 0));

    // shoot flywheel with limelight
    shootToTarget.whenActive(new SmartShoot(flywheel, hopper)).whenInactive(new SetFlywheelRPM(flywheel, 0)).whenInactive(new HopperManual(hopper, 0, 0));

    resetHopper.whenPressed(new ResetBallsHopper(hopper));

    // hopper.setDefaultCommand(new HopperAutomatic(hopper, intake));

    // alignShoot.whenPressed(new AlignShoot(flywheel, hopper, drivetrain));

    hopperUp.whenPressed(new HopperManual(hopper, 0.25, 0.25)).whenReleased(new HopperManual(hopper, 0, 0));
    // hopperDown.whenPressed(new HopperManual(hopper, -0.25, -0.25)).whenReleased(new HopperManual(hopper, 0, 0));

    // shoot flywheel at set rpm
    //  new JoystickButton(joystick, 7).whenPressed(new SetFlywheelRPM(flywheel, 6000))
    //      .whenReleased(new SetFlywheelRPM(flywheel, 0));

    // auto hpper
    autoHopper.whenPressed(new HopperAutomatic(hopper, intake)).whenReleased(new HopperManual(hopper, 0, 0)).whenPressed(new RunIntake(intake, true)).whenReleased(new RunIntake(intake, false));
    // Align robot to target
    // autoAlign.whenPressed(new AutoAlign(drivetrain)).whenReleased(new StopDrivetrain(drivetrain));
    // adithya patil was here
    // jacob gino was also here
    // Deruiter is goated
    // Codingbat for life. Lets go commonEnd.java is the best excerise
    // Vincent dont forget to do APUSH and Conlin hw

  }

  /**
   * Gets the throttle from the trigger on the # axis, or __ side trigger
   * 
   * @return double from -1 to 1 representing power going back or forth
   */
  public double getThrottle() {
    // aDiTyA : 0.7, axis 5
    // AbHiK: 0.6, axis 1
    // jAcOb: 0.7 axis 5
    throttle.updateValue(-joystick.getRawAxis(5) * 0.7);
    return throttle.getAverage();
  }

  /**
   * 
   * Gets the wheel from the trigger on the # axis, or __ side trigger
   * 
   * @return double from -1 to 1 representing power towards turning
   */
  public double getWheel() {
    // aditya + jAcOb: axis 0
    // abhik: axis 4
    wheel.updateValue(joystick.getRawAxis(0) * 0.8);
    return wheel.getAverage();
  }

  /**
   * Gets the status of the quickTurnButton
   * 
   * @return true if button for quickTurn has been pressed, false otherwise
   */
  public boolean getQuickTurn() {
    return quickturn.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // autonSelector = new SendableChooser<>();

    // autonSelector.setDefaultOption("Basic Shoot", new AutonRoutine3());
    // autonSelector.addOption("Trench Run", new AutonRoutine(intake, hopper, drivetrain, flywheel));
    // autonSelector.addOption("Rendezvous Run", new AutonRoutine2());
    // autonSelector.addOption("Rendezvous Run Small", new RendezvousAuton2());
    // autonSelector.addOption("Shoot then Back", new BasicAuto());
    // SmartDashboard.putData(autonSelector);
    
    return new RunDrivetrain(drivetrain, -0.5).andThen(new Wait(2)).andThen(new RunDrivetrain(drivetrain, 0)).andThen(new SmartShoot(flywheel, hopper));
  
  }
}
