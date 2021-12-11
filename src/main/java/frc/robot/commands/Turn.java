// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Part of MultiplePaths - Turn Command
// Credit where it's due: karenlyuan, anikay1807, krishshah13

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Turn extends CommandBase {

  private DriveTrain d;
  private double angle;
  private double constant = 0.01;

  /** Creates a new EncoderDrive. */
  public Turn(DriveTrain d, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.d = d;
    this.angle = angle;
    //this.power = (angle >= 0 ? power : power*-1.0);
    //this.power = power;
    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    d.navxReset();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = angle - d.getAngle();
    double power = error * constant;
    d.tankDrive(power, -1.0 * power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    d.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(d.getAngle() - angle) <= 0.1);
  }

}