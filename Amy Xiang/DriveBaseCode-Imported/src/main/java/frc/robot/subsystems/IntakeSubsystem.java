/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SpeedControllerGroup SCG_Intake = new SpeedControllerGroup(
                                                  new WPI_TalonSRX(IntakeConstants.INTAKE_ID_1), 
                                                  new WPI_TalonSRX(IntakeConstants.INTAKE_ID_2));

  /**
   * Creates a new ExampleSubsystem.
   */
  public IntakeSubsystem() {
   
  }
  public void spinIntake() {
    SCG_Intake.set(IntakeConstants.INTAKE_SPEED);
  }

  public void stopIntake() {
    SCG_Intake.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
