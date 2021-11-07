// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.RobotContainer;
import frc.robot.commands.DriveLine;
import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class SetVelocity extends CommandBase {
  /** Creates a new SetFlyWheelVelocity. */

  private PIDController TalonPID;
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  // private SimpleMotorFeedforward FeedForward;
  // private double kS = 0.0;
  // private double kCos = 0.0;
  // private double kV = 0.0;
  // private double kA = 0.0;

  // private double kTolerance = 0.0;
  // private double kDerivativeTolerance = 0.0;

  private static double velocitySetPoint = 0.0;

  public SetVelocity(double v) {
    // Use addRequirements() here to declare subsystem dependencies.
    velocitySetPoint = v;
    TalonPID = new PIDController(kP, kI, kD);
  }

  public static void set(double v) {
    velocitySetPoint = v;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TalonPID.setSetpoint(velocitySetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   DriveLine.setPower(TalonPID.calculate(DriveLine.getAverageVelocity()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {isFinished();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
