// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class DriveManually extends CommandBase {
  /** Creates a new DriveManually. */
  public DriveManually() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveSubsystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  //actually runs subsystem
  public void execute() {
    double leftStick = -Robot.m_robotContainer.m_driverController.getLeftY();
    double rightStick = -Robot.m_robotContainer.m_driverController.getRightY();
    Robot.driveSubsystem.teleop(leftStick, rightStick);
  }

  // Called once the command end s or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
