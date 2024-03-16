package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
private DriveSubsystem m_drive;


@Override
public void forward {
    m_drive.setDriveSpeed_Arcade(-m_xSpeed.getAsDouble(), m_zRotation.getAsDouble());
}
