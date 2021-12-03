// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonSRX leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
  private WPI_TalonSRX rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);


  /** Creates a new DriveTrain. */
  public DriveTrain() {

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(false);

    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(ControlMode.PercentOutput, rightSpeed);
    leftDriveTalon.set(ControlMode.PercentOutput, leftSpeed);
  }
  public double getAverageEncoderPosition() {
    return (leftDriveTalon.getSelectedSensorPosition() + rightDriveTalon.getSelectedSensorPosition())/2.0;
  }
  public void resetEncoders() { 
    leftDriveTalon.setSelectedSensorPosition(0.0);
    rightDriveTalon.setSelectedSensorPosition(0.0);
  }
}
