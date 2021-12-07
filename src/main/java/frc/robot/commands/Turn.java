// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Part of MultiplePaths - Forward Command
// Credit where it's due: karenlyuan, anikay1807, krishshah13

package frc.robot.commands;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import com.kauailabs.navx.frc.AHRS;

public class Turn extends CommandBase {

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private DriveTrain d;
  private double angle;
  private double power;
  private int constant = 1;

  /** Creates a new EncoderDrive. */
  public Turn(DriveTrain d, double angle, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.d = d;
    this.angle = angle;
    this.power = power;

    if(angle < 0) {
      constant = -1;
    }

    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    navx.reset();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    d.tankDrive(constant * Math.abs(power), -constant * Math.abs(power));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    d.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(d.getAngle()) >= Math.abs(angle));
  }

}