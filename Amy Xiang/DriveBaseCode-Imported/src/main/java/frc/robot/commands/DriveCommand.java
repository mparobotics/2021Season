/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class DriveCommand extends CommandBase {
  private DriveSubsystem m_subsystem;
  private XboxController m_XboxController;
  
  /**
   * Creates a new DriveCommand.
   */
  public DriveCommand(DriveSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_XboxController = controller; 

    addRequirements(subsystem); 
  
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.tankDrive(m_XboxController.getY(Hand.kLeft), m_XboxController.getY(Hand.kRight));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
