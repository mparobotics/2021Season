// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveManually;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class Container {
  public XboxController m_driverController = new XboxController(0);
  public static AnalogGyro m_gyro = new AnalogGyro(ConstantsMap.GyroPort);

  // The robot's subsystems and commands are defined here...
  public DriveSubsystem DriveSub = new DriveSubsystem();
  public DriveManually DriveMan = new DriveManually();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Container() {
    
    configureButtonBindings();
    DriveSub.setDefaultCommand(new DriveManually ());


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    //XboxController.Button.kLeftBumper.whenPressed(new DriveForward());
    //new JoystickButton(m_driverController, Button.kA.value).whenPressed(new DriveForward());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
