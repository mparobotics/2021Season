// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrajDrive extends SubsystemBase {
  /** Creates a new TrajDrive. */
  public TrajDrive() {}

  public static SequentialCommandGroup DoTrajDrive() {

    final DriveTrain drive = new DriveTrain();
      //LOAD A PATH FILE HERE
      double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      boolean PathToTake;
      if (tv < 1) {
        PathToTake = false;
        
          }
          else {PathToTake = true;}
      
          SmartDashboard.putBoolean("limelight vision", PathToTake);
  
  
      if (PathToTake = true){
      String trajectoryJSON = "paths/RedPath.wpilib.json";
      Trajectory trajectory = new Trajectory();
      
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        System.out.println("Sucsess!");
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }
      RamseteCommand command =
      new RamseteCommand(
          trajectory,
          drive::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
              DriveConstants.kS,
              DriveConstants.kV,
              DriveConstants.kA),
          DriveConstants.kDriveKinematics,
          drive::getWheelSpeeds,
          new PIDController(DriveConstants.kP, 0, 0),
          new PIDController(DriveConstants.kP, 0, 0),
          // RamseteCommand passes volts to the callback
          drive::tankDriveVolts,
          drive);
  
  // Reset odometry to the starting pose of the trajectory.
  drive.resetOdometry(trajectory.getInitialPose());
  
  // Run path following command, then stop at the end.
  return command.andThen(() -> drive.tankDriveVolts(0, 0));
    
    
    }
      else {String trajectoryJSON = "paths/BluePath.wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Sucsess!");
            } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }            RamseteCommand command =
      new RamseteCommand(
          trajectory,
          drive::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
              DriveConstants.kS,
              DriveConstants.kV,
              DriveConstants.kA),
          DriveConstants.kDriveKinematics,
          drive::getWheelSpeeds,
          new PIDController(DriveConstants.kP, 0, 0),
          new PIDController(DriveConstants.kP, 0, 0),
          // RamseteCommand passes volts to the callback
          drive::tankDriveVolts,
          drive);
  
  // Reset odometry to the starting pose of the trajectory.
  drive.resetOdometry(trajectory.getInitialPose());
  
  // Run path following command, then stop at the end.
  
  return command.andThen(() -> drive.tankDriveVolts(0, 0));}
      
  //END OF LOADING PATH FILE
  
      // An ExampleCommand will run in autonomous
    
  }

  
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
