/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArcadeDriveClassic;

import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 * 
 * Contains subsystems, OI devices, and commands (runs commands)
 */
public class RobotContainer {
  // declaring and intializing subsystems(all) and commands (most commands don't need to be delclared/intialized here)
  public DriveSubsystem driveSub      = new DriveSubsystem();
 


  // declaring and intializing controller(s)
  //private XboxController xbox = new XboxController(OIConstants.XBOX_ID);
  //private XboxController helms = new XboxController(OIConstants.HELMS_ID);
  public static Joystick leftJoystick = new Joystick(OIConstants.left_Joystick);
  public static Joystick rightJoystick = new Joystick(OIConstants.right_Joystick);

  public static boolean shooting = false;
   
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //drive controls
    /*driveSub.setDefaultCommand(new ArcadeDriveTrigger(driveSub, 
                              () -> xbox.getTriggerAxis(Hand.kLeft), 
                              () -> xbox.getTriggerAxis(Hand.kRight),
                              () -> xbox.getX(Hand.kLeft)));*/
   driveSub.setDefaultCommand(new ArcadeDriveClassic(driveSub,
                              () -> leftJoystick.getY(),
                              () -> rightJoystick.getY()));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Main's Commands - Winch is right stick, 
    

    //Helm's Commands
  }

  public Joystick getController() {
    return leftJoystick; 

  }

  public XboxController getHelms() {
    return getHelms();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * currently there is no auto command
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
