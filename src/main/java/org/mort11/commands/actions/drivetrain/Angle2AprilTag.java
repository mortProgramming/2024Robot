package org.mort11.commands.actions.drivetrain;

import org.mort11.configuration.Odometer;
import org.mort11.subsystems.Drivetrain;
import org.mort11.subsystems.LimelightHelpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class Angle2AprilTag extends Command{

    //declare private drivetrain instance
    private Drivetrain drivetrain;
    // private NetworkTable limelightTable;
    //variable stores wanted angle
    

    //initializes command with wanted angle
    public Angle2AprilTag(double wantedAngle){

        //gets singleton instance of drivetrain
        drivetrain = Drivetrain.getInstance();
        // limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        //sets the wanted angle
        
        addRequirements(drivetrain);
    }

    //executes the command
    @Override
    public void execute(){
        //gets yaw angle tx from the limelight
        // double wantedAngle = limelightTable.getEntry("tx").getDouble(0);
        double wantedAngle = LimelightHelpers.getTX("limelight");

        //uses the yaw angle to rotate to wanted angle
        drivetrain.setAngle2Controller(wantedAngle);
        System.out.println(wantedAngle);
    }
    
    @Override
    public void end(boolean interrupted){
        drivetrain.setDrive(new ChassisSpeeds(0,0,0));
    }

    @Override
    public boolean isFinished(){
        return drivetrain.getRotateControllerAtSetpoint();
    }   
}
