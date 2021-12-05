// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDTurn extends PIDCommand {
  /** Creates a new PIDTurn. */
  private DriveTrain driveTrain;
  public PIDTurn(DriveTrain d, double angle) {
    super(
        // The controller that the command will use
        // Power over angle is the rough constant?
        new PIDController(0.5/angle, 0, 0),
        // This should return the measurement
        d::getAngle,
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        output -> {
          d.tankDrive(output, -output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    driveTrain = d;
    addRequirements(d);
    getController().setTolerance(1.0);
  }
  @Override
  public void initialize() {
    driveTrain.navxReset();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
