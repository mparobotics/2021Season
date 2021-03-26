// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TrajDrive;

public class AutoCommand extends CommandBase {
  private TrajDrive m_autoSub;
  /** Creates a new AutoCommand. */
  public AutoCommand(TrajDrive autoSub) {
    
      m_autoSub = autoSub;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_autoSub);
    }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TrajDrive.DoTrajDrive();
    }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
