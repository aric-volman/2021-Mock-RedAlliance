// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  /** Creates a new DriveDistance. */
  DriveTrain _driveTrain;
  boolean _finished;

  //This variable stores the calculated amount of encoder ticks needed to reach the specified distance

  double _distance;
  double _drivespeed;

  public DriveDistance(DriveTrain driveTrain, double distance, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = driveTrain;
    addRequirements(_driveTrain);
    _distance = distance;
    _drivespeed = (_distance >= 0 ? 1 : -1) * Math.abs(speed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _finished = false;
    _driveTrain.tankDrive(0.0, 0.0);
    _driveTrain.zeroDisplacement();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(_driveTrain.getAverageDisplacement()) < Math.abs(_distance)) {
      _driveTrain.tankDrive(_drivespeed, _drivespeed);
    }
    else {
      _finished = true;
      _driveTrain.tankDrive(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _finished;
  }

}