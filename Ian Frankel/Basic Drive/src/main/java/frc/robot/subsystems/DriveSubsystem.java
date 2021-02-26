// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsMap;
import frc.robot.commands.DriveManually;

public class DriveSubsystem extends SubsystemBase {
  //instantiate new motor control objects
  public WPI_TalonSRX leftFront = new WPI_TalonSRX(ConstantsMap.leftFrontPort);
  public WPI_TalonSRX leftBack = new WPI_TalonSRX(ConstantsMap.leftBackPort);
  public WPI_TalonSRX rightFront = new WPI_TalonSRX(ConstantsMap.rightFrontPort);
  public WPI_TalonSRX rightBack = new WPI_TalonSRX(ConstantsMap.rightBackPort);

  //instantiate a new DiferentialDrive object
  //assign motor controlers to differentialDrive
  public DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront); 

  //create constructor function 
  public DriveSubsystem() {
      //point back motors to do the same as the front motors
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);  
  }

    
  //add teleop() method  
  public void teleop(double leftStick, double rightStick ) {
    //speed reducer
    leftStick = leftStick / 1.75;
    rightStick = rightStick / 1.75;
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
