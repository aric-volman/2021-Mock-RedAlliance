// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.TankDrive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveLine;
import frc.robot.commands.PIDTurn;
import frc.robot.commands.Turn;
// import frc.robot.commands.MultiplePaths;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveTrain driveTrain;
  private final DriveLine driveLine;
  private final DriveDistance driveDistance;
  private final PIDTurn turnNinetyPID;
  private final Turn turnNine;
  private SequentialCommandGroup multiplePaths;

  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private final TankDrive tankDrive;

  String gameData; 
  
  private final double drivePower = 0.1;
  private final double turnPower = 0.1;

  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Drivetrain
    driveTrain = new DriveTrain();

    // Autonomous Showcase Portion
    driveLine = new DriveLine(driveTrain, 2.0, drivePower);
    driveDistance = new DriveDistance(driveTrain, 2.0, drivePower);
    turnNinetyPID = new PIDTurn(driveTrain, -90.0, turnPower);
    turnNine = new Turn(driveTrain, -90.0, turnPower);
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B' :
          SmartDashboard.putString("Path Color", "Blue");
          multiplePaths = new SequentialCommandGroup(
            new DriveDistance(driveTrain, 1.0, drivePower), 
            new Turn(driveTrain, 90.0, turnPower), 
            new DriveDistance(driveTrain, 1.0, drivePower));
            break;
        case 'R' :
          SmartDashboard.putString("Path Color", "Red");
          multiplePaths = new SequentialCommandGroup(
            new DriveDistance(driveTrain, 1.0, drivePower),
            new Turn(driveTrain, 90.0, turnPower),
            new DriveDistance(driveTrain, 1.0, drivePower),
            new Turn(driveTrain, 90.0, turnPower),
            new DriveDistance(driveTrain, 1.0, drivePower),
            new Turn(driveTrain, 90.0, turnPower),
            new DriveDistance(driveTrain, 1.0, drivePower));
            break;
        case 'Y' :
          SmartDashboard.putString("Path Color", "Yellow");
          multiplePaths = new SequentialCommandGroup(
            new DriveDistance(driveTrain, 0.5, drivePower),
            new Turn(driveTrain, -90.0, turnPower),
            new DriveDistance(driveTrain, -0.5, drivePower));
            break;
        default :
          multiplePaths = new SequentialCommandGroup();
          break;
      }
    }
    // SendableChooser
    chooser.addOption("Drive Distance", driveDistance);
    chooser.addOption("Drive To Line", driveLine);
    chooser.addOption("Multiple Paths", multiplePaths);
    chooser.addOption("Turn 90 degrees to the left (without PID)", turnNine);
    chooser.addOption("Turn 90 degrees to the left (with PID)", turnNinetyPID);

    SmartDashboard.putData(chooser);

    // Teleop Portion
    leftJoystick = new Joystick(Constants.USBOrder.Zero);
    rightJoystick = new Joystick(Constants.USBOrder.One);
    tankDrive = new TankDrive(driveTrain, leftJoystick, rightJoystick);
   
    driveTrain.setDefaultCommand(tankDrive);
    
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // JoystickButton joystickDriveToLine = new JoystickButton(leftJoystick, 6);
    // joystickDriveToLine.whenPressed(driveLine);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
  }
}