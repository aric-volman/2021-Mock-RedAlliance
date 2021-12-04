// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Part of MultiplePaths - SequentialCommandGroup with Logic
// Credit where it's due: karenlyuan and anikay1807 for the paths

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MultiplePaths extends SequentialCommandGroup {
  /** Creates a new MultiplePaths. */
  DriveTrain dt;
  String gameData;

  public MultiplePaths(DriveTrain dt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.dt = dt;
    gameData = DriverStation.getInstance().getGameSpecificMessage();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B' :
        // Blue case code
        // TODO - Drive a meter, then turn right, then drive a meter
        break;
      case 'R' :
        addCommands(
          new Forward(dt, 1, 0.7),
          new Turn(dt, 90, 0.7),
          new Forward(dt, 1, 0.7),
          new Turn(dt, 90, 0.7),
          new Forward(dt, 1, 0.7),
          new Turn(dt, 90, 0.7),
          new Forward(dt, 1, 0.7));
        break;
      case 'Y' :
        // Yellow case code
        addCommands(
          new Forward(dt, 0.5, 0.7),
          new Turn(dt, 90, 0.7),
          new Forward(dt, -0.5, 0.7));
        break;
      default :
        // This is corrupt data
        break;
      }
    }
  }
}
