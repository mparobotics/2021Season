package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ConstantsMap;
import frc.robot.Container;
import frc.robot.Robot;



public class DriveForward extends CommandBase {
    /** Creates a new DriveManually. */
    public DriveForward() {
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
      double leftStick = -Robot.m_robotContainer.m_driverController.getY(Hand.kLeft);
      double turningValue = (ConstantsMap.kAngleSetpoint - Container.m_gyro.getAngle()) * ConstantsMap.kP;
    // Invert the direction of the turn if we are going backwards
    turningValue = Math.copySign(turningValue, leftStick);
    
      Robot.driveSubsystem.goFoward(leftStick, turningValue);
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