/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonRoutine2 extends SequentialCommandGroup {
  Intake intake;
  Hopper hopper;
  Drivetrain drivetrain;
  Flywheel flywheel;
  /**
   * Rendezvous Auton Routine
   */
  public AutonRoutine2(Intake intake, Hopper hopper, Drivetrain drivetrain, Flywheel flywheel) {
    this.intake = intake;
    this.hopper = hopper;
    this.drivetrain = drivetrain;
    this.flywheel = flywheel;

    addCommands(
        new ParallelRaceGroup(    
            new SequentialCommandGroup(
              new ParallelCommandGroup(
                getTrajectory1(),
                new RunIntake(intake, true).withTimeout(3)
              ),
              new ParallelRaceGroup(
                getTrajectory2(),
                new RunIntake(intake, true)
            )
          ),
          new SmartShoot(flywheel, hopper).withTimeout(15)
        ),
        new SmartShoot(flywheel, hopper).withTimeout(4),
        new ParallelRaceGroup(  
            new ParallelCommandGroup(
              getTrajectory3(),
              new RunIntake(intake, true).withTimeout(4.5)
            ),
            new HopperAutomatic(hopper, intake)
        )
      );
  }

  public Command getTrajectory1(){
    drivetrain.invertPathDirection(true);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(),
      new Pose2d(-2.05, -.2, new Rotation2d(.4,.161))
     // new Pose2d(-2, -.2, new Rotation2d(1.52, .62))
    ), drivetrain.getTrajectoryConfig());

    
    return drivetrain.getRamseteCommand(traj1);
  }

  public Command getTrajectory2(){
    drivetrain.invertPathDirection(false);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(

      // new Pose2d(-2, -.2, new Rotation2d(2.99, 1.4)),
      // new Pose2d(-.4, -1.8, new Rotation2d(2.2,0))

      new Pose2d(-2, -.2, new Rotation2d(.4,.161)),
      new Pose2d(-.9, -.6, new Rotation2d(0,-.1)),
      new Pose2d(-.4,-2, new Rotation2d().fromDegrees(-10))

    ), drivetrain.getTrajectoryConfigSlow());

    return drivetrain.getRamseteCommand(traj1);
  }

  public Command getTrajectory3(){
    drivetrain.invertPathDirection(true);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(-.4,-2, new Rotation2d().fromDegrees(0)),
      new Pose2d(-2.4, -1.6, new Rotation2d(.374, -.55))

    ), drivetrain.getTrajectoryConfig());

    return drivetrain.getRamseteCommand(traj1);
  }
}