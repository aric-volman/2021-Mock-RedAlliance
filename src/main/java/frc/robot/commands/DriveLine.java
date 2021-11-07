// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.SetVelocity;

public class DriveLine extends CommandBase {
  /** Creates a new TimedDrive. */
  private static DriveTrain driveTrain;
  private final Timer timer;
  private double velocity;
  private double distance;
  private double time;

  /*
   * Converting meters to rotations involves dividing by (2.5/2)^2*pi/39.37, which
   * is 0.124, so divide 1 by 0.124, which is now 8.02 rotations per meter
   */
  public DriveLine(DriveTrain dt, double v, double d) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    timer = new Timer();
    distance = d;
    velocity = v;
    time = (8.02 * distance) / velocity;
    addRequirements(driveTrain);
  }

  public static double getAverageVelocity() {
    return driveTrain.getCombinedAverageVelocityInRotationsPerSecond();
  }

  public static void setPower(double p) {
    driveTrain.tankDrive(p, p);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() <= time) {
      SetVelocity.set(velocity);
    } else {
      SetVelocity.set(0.0);
      // driveTrain.tankDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
/*
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= 5);
  
}*/
}
