// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
// import frc.robot.RobotContainer;

public class Floppy extends SubsystemBase {

  private WPI_TalonSRX floppyTalon; 

  /** Creates a new Floppy. */
  public Floppy() {
    floppyTalon = new WPI_TalonSRX(Constants.MechanismPorts.Floppy);
    floppyTalon.configFactoryDefault();
    floppyTalon.setInverted(false);
    floppyTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
  }

  public void setFloppyPower(double power) {
    floppyTalon.set(ControlMode.PercentOutput, power);
  }

  public double getFloppyAngle() {
    return floppyTalon.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}