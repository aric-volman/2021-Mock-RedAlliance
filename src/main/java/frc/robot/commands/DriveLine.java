package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;

public class DriveLine extends CommandBase {
    
    private DriveTrain driveTrain;

    private double givenPower;
    private double totalEncoderTicksNeeded;
    private double distance;
    private boolean finished = false;

    public DriveLine(DriveTrain dt, double d, double p) { 
        driveTrain = dt; 
        // givenPower = (d >= 0.0 ? 1.0 : -1.0) * Math.abs(p);
        givenPower = p;
        distance = d;

        /*
        Explanation:
        6 inches is the wheel's diameter. The conversion factor 
        from inches to meters is around 0.0254. We can get the 
        circumference in meters this way by multiplying by pi
        and the conversion ratio. If we divide the total unit of 
        length (1.0 meter) by this circumference, we get the rotations per 
        1 meter, which turns out to be 2.08864755 rotations per meter.
        If we know the number of rotations per meter, the number of ticks per rotation, 
        and the distance in meters, we can find the number of ticks 
        per distance in meters.
        */
        
        totalEncoderTicksNeeded = Constants.DriveToLineConstants.rotationsPerMeterByTicks*distance;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        //driveTrain.resetEncoders();
        // Note: The constructor runs once, initialize() runs every schedule, so this resets it (finished).
        finished = false;
        SmartDashboard.putString("Status", "Starting");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       // if (Math.abs(driveTrain.getAverageEncoderPosition()) <= Math.abs(totalEncoderTicksNeeded)) {
        double tickOutput = (driveTrain.getRightEncoderCount() + driveTrain.getLeftEncoderCount())/2.0;
        if (tickOutput <= totalEncoderTicksNeeded) {
            SmartDashboard.putString("Status", "In Progress");
            SmartDashboard.putNumber("Encoder Position", tickOutput);
            SmartDashboard.putNumber("Target Ticks", totalEncoderTicksNeeded);
            System.out.println(totalEncoderTicksNeeded);
            System.out.println(tickOutput);
            driveTrain.tankDrive(givenPower, givenPower);
            tickOutput = (driveTrain.getRightEncoderCount() + driveTrain.getLeftEncoderCount())/2.0;
        } else {
            driveTrain.tankDrive(0.0, 0.0);
            finished = true;
            SmartDashboard.putString("Status", "Finished");
        }
    }
    @Override
    public void end(boolean interrupted) {SmartDashboard.putString("Status", "Finished"); driveTrain.tankDrive(0.0, 0.0);}
    @Override
    public boolean isFinished() {
        return finished;
    }
}