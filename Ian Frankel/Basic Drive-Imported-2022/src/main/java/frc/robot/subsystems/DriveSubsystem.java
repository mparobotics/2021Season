// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsMap;
import frc.robot.commands.DriveManually;

public class DriveSubsystem extends SubsystemBase {
  //instantiate new motor control objects
  private final WPI_TalonSRX leftBack = new WPI_TalonSRX(ConstantsMap.FALCON_BL_ID); 
  private final WPI_TalonSRX leftFront = new WPI_TalonSRX(ConstantsMap.FALCON_FL_ID); 
  private final WPI_TalonSRX rightBack = new WPI_TalonSRX(ConstantsMap.FALCON_BR_ID); 
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(ConstantsMap.FALCON_FR_ID); 


  private final SpeedControllerGroup SCG_R = new SpeedControllerGroup(rightFront, rightBack); 
  private final SpeedControllerGroup SCG_L = new SpeedControllerGroup(leftFront, leftBack);   
  //instantiate a new DiferentialDrive object
  //assign motor controlers to differentialDrive
  private final DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);

  //create constructor function 
  public DriveSubsystem() {
      //point back motors to do the same as the front motors
      rightFront.setNeutralMode(NeutralMode.Brake);
      leftFront.setNeutralMode(NeutralMode.Brake);
      rightBack.setNeutralMode(NeutralMode.Brake);
      leftBack.setNeutralMode(NeutralMode.Brake);
      rightBack.follow(rightFront); //talonBR follows TalonFR
      leftBack.follow(leftFront); //talonBL follows TalonFR 
          //falconFR.setInverted(true); //set to invert falconFR.. CW/CCW.. Green = forward (motor led)
      rightBack.setInverted(InvertType.FollowMaster); //matches whatever falconFR is

    //falconFL.setInverted(true); //set to invert falconFL.. CW/CCW.. Green = foward (motor led)
      leftBack.setInverted(InvertType.FollowMaster); //matches whatever falcon FL is
  }

    
  //add teleop() method  
  public void teleop(double leftStick, double rightStick ) {
    //speed reducer
    leftStick = leftStick / 1.25;
    rightStick = rightStick / 1.25;
    //drive command
    drive.tankDrive(leftStick, rightStick);
  }

  public void goFoward(double leftStick, double turningValue) {
     drive.arcadeDrive(leftStick, turningValue);

  }



  
  @Override
  public void periodic() {
    setDefaultCommand(new DriveManually()); 
  }
}
