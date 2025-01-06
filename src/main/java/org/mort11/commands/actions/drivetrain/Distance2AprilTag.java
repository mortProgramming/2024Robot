package org.mort11.commands.actions.drivetrain;

import org.mort11.subsystems.Drivetrain;
import org.mort11.subsystems.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Distance2AprilTag {
    public double getLimeLightDistance() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 35.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 10.5; 

    // distance from the target to the floor
    double goalHeightInches = 19.0; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        
    // SmartDashboard.putNumber("limelight distance", distanceFromLimelightToGoalInches);

    return distanceFromLimelightToGoalInches;


}


   
    }
    





    