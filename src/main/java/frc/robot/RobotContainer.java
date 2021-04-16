// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberManualCommand;
import frc.robot.subsystems.Climber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Joystick operatorJoystick;

  private final Climber climber = new Climber();
  private final Trigger climberTrigger;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    operatorJoystick = new Joystick(1);

    // Configure the button bindings
    configureButtonBindings();

    climberTrigger = new Trigger();
    climberTrigger.whenActive(new ClimberManualCommand(climber));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(joystick, 1).whenPressed(new ExampleCommand(m_exampleSubsystem, true));

    // if you want to run a command when the button is released
    SmartDashboard.putData("Manually Move Climber Down", new ClimberDownCommand(climber));
    SmartDashboard.putData("Move Climber Up", new ClimberUpCommand(climber));

    new JoystickButton(operatorJoystick, 1).whenPressed(new ClimberDownCommand(climber));
    new JoystickButton(operatorJoystick, 2).whenPressed(new ClimberUpCommand(climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public double getLeft()
  {
    return operatorJoystick.getRawAxis(1);
  }

  public Climber getClimber()
  {
    return climber;
  }
}
