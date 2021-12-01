package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveLine extends CommandBase {
    
    private DriveTrain driveTrain;

    // Note: For anything that's not an object, don't use final with DiffDrive
    private double givenPower;
    private double calculatedEncoderTicks;
    private double distance;
    private boolean finished;

    public DriveLine(DriveTrain dt, double p, double d) { 
        driveTrain = dt; 
        givenPower = p;
        distance = d;

        /*
        Explanation:
        2.5 inches is the wheel's diameter. The conversion factor 
        from inches to meters is around 0.0254. We can get the 
        circumference in meters this way. If we divide the total 
        unit of length (1.0 meter) by this, we get the rotations per 
        1 meter, which turns out to be 5.01275411 rotations per meter.
        If we know the number of rotations per meter, the number of ticks per rotation, 
        and the distance in meters, we can find the number of ticks 
        per distance in meters.
        */
        
        calculatedEncoderTicks = Constants.DriveToLineConstants.rotationsPerMeterByTicks*distance;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetEncoders();
        // Note: The constructor runs once, initialize() runs every schedule, so this resets it (finished).
        finished = false;
        SmartDashboard.putString("Status", "Starting");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (driveTrain.getAverageEncoderPosition() <= calculatedEncoderTicks) {
          SmartDashboard.putString("Status", "In Progress");
            driveTrain.tankDrive(givenPower, givenPower);
        } else {
            driveTrain.tankDrive(0.0, 0.0);
            finished = true;
            SmartDashboard.putString("Status", "Finished");
        }
    }
    @Override
    public void end(boolean interrupted) {driveTrain.tankDrive(0.0, 0.0);}
    @Override
    public boolean isFinished() {
        return finished;
    }
}