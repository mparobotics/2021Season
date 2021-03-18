/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
/**
 * this subsystem sets up and directly manipulates everything on the drive train
 */
public class DriveSubsystem extends SubsystemBase {

  //declaring and intializing drive motor controllers and assciated configuration objects
  private final static WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.FALCON_BL_ID);
  private final static WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.FALCON_FL_ID);
  private final static WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.FALCON_BR_ID);
  private final static WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.FALCON_FR_ID);

  private final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  private final static SpeedControllerGroup SCG_R = new SpeedControllerGroup(falconFR, falconBR);
  private final static SpeedControllerGroup SCG_L = new SpeedControllerGroup(falconFL, falconBL);

  private final static DifferentialDrive drive = new DifferentialDrive(SCG_L, SCG_R);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    setBrake();
    // setting ramp

    // Drive Base Code
    falconBR.follow(falconFR); // talonBR follows TalonFR
    falconBL.follow(falconFL); // talonBL follows TalonFR

    // falconFR.setInverted(true); //set to invert falconFR.. CW/CCW.. Green =
    // forward (motor led)
    falconBR.setInverted(InvertType.FollowMaster); // matches whatever falconFR is

    // falconFL.setInverted(true); //set to invert falconFL.. CW/CCW.. Green =
    // foward (motor led)
    falconBL.setInverted(InvertType.FollowMaster); // matches whatever falcon FL is
    // Encoder Code Start
    fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; // Selecting Feedback Sensor

  }

  public void setBrake() {
    // setting coast or brake mode, can also be done in Phoenix tuner
    falconFR.setNeutralMode(NeutralMode.Brake);
    falconFL.setNeutralMode(NeutralMode.Brake);
    falconBR.setNeutralMode(NeutralMode.Brake);
    falconBL.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * sets the speed of the drive train with arcade controls
   * 
   * @param xSpeed
   * @param zRotation
   */


  /**
   * set the speed of the drive train with tank controls (WIP)
   * 
   * /** stops the drive train
   */
  public void stopRobot() {
    falconFR.set(ControlMode.PercentOutput, 0);
    falconFL.set(ControlMode.PercentOutput, 0);
  }

  public static void teleop(double leftStick, double rightStick) {
    // speed reducer
    //leftStick = leftStick / 1.25;
    //rightStick = rightStick / 1.25;
    // drive command
    drive.tankDrive(leftStick, rightStick);
  }


  /**
   * prints encoder values to the smart dashboard
   */



  public void encoderReset() {
    falconFR.setSelectedSensorPosition(0);
    falconFL.setSelectedSensorPosition(0);
    falconBR.setSelectedSensorPosition(0);
    falconBL.setSelectedSensorPosition(0);
  }
}