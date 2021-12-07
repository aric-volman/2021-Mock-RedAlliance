// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonSRX leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
  private WPI_TalonSRX rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private double circumference = 1;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);

    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

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
    leftDriveTalon.setSelectedSensorPosition(0.0, 0, 10);
    rightDriveTalon.setSelectedSensorPosition(0.0, 0, 10);
  }

  // Part of MultiplePaths - DriveTrain Methods
  // Credit where it's due: karenlyuan, anikay1807, krishshah13
  
  public double getPosition() {
    return ((leftDriveTalon.getSelectedSensorPosition() + rightDriveTalon.getSelectedSensorPosition())/2) * (circumference/4096);
    //average distance of both left and right
  }

  public double getVelocity(){
    return ((leftDriveTalon.getSensorCollection().getPulseWidthVelocity() + rightDriveTalon.getSensorCollection().getPulseWidthVelocity())/2) * (circumference/4096);
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public void navxReset() {
    navx.reset();
  }

  // For DriveDistance to work
  private double getLeftEncoderCount() {
    return leftDriveTalon.getSelectedSensorPosition();
  }

  private double getRightEncoderCount() {
    return rightDriveTalon.getSelectedSensorPosition();
  }

  private double getAverageEncoderCount() {
    return (getLeftEncoderCount() + getRightEncoderCount()) / 2.0;
  }

  public double getAverageDisplacement() {
    return getAverageEncoderCount() * Constants.DriveToLineConstants.MetersPerEncoderTick;
  }

  public void zeroDisplacement() {
    resetEncoders();
  }
}
