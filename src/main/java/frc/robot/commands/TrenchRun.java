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
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Hopper;
import frc.robot.utils.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TrenchRun extends SequentialCommandGroup {
  Drivetrain drivetrain;
  Hopper hopper;
  /**
   * Right Side Trench Auton
   */
  public TrenchRun(Intake intake, Hopper hopper, Drivetrain drivetrain, Flywheel flywheel, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.hopper = hopper;
    addCommands(
      // shoot at start
      // new SmartShoot(flywheel, hopper, limelight, true).withTimeout(10),
      // intake at trench
      new ParallelRaceGroup(
        new ParallelCommandGroup(
          getTrajectory1(),
          new RunIntake(intake, true).withTimeout(5.5).andThen(new RunIntake(intake, false))
        ),
        new HopperAutomatic(hopper, intake).withTimeout(10)
      ),
      // go back and shoot
      // new ParallelRaceGroup(
      //   new SequentialCommandGroup(
          getTrajectory2()
          // new SmartShoot(flywheel, hopper, limelight, true).withTimeout(5)
        // ),
      //   new RunIntake(intake, true).withTimeout(7)
      // )
    );
  }

  public Command getTrajectory2(){
    
    drivetrain.invertPathDirection(false);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(-4, -1.55, new Rotation2d()),  
      new Pose2d(-1.25,0, Rotation2d.fromDegrees(8))

    ), drivetrain.getTrajectoryConfig());

    
    // String trajectoryJSON = "paths/Right-Path2.wpilib.json";
    // Path trajectoryPath;
    // try {
    //   trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //   TrajectoryUtil.toPathweaverJson(traj1, trajectoryPath);
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
 //   }
 
    return drivetrain.getRamseteCommand(traj1);
  }

  public Command getTrajectory1(){
    drivetrain.invertPathDirection(true);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(
      
      List.of(   
        // new Pose2d(0, 0, new Rotation2d()),     
        new Pose2d(-2.2,-1.55, Rotation2d.fromDegrees(8)),
        new Pose2d(-4, -1.55, new Rotation2d())
      ), 
      
      drivetrain.getTrajectoryConfig()
    );

    return drivetrain.getRamseteCommand(traj1);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();

    SmartDashboard.putString("Auton", "Auton 1 Running");
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();

    hopper.setBallsInHopper(3);
  }
}