package org.mort11.commands.actions.drivetrain;

import org.mort11.subsystems.LimelightHelpers;
import org.mort11.subsystems.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.geometry.Pose3d;

public class Distance2AprilTag {

    public double getMegaTagDistance() {
        PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (botPoseEstimate == null) {
            return 0;
        }
        double x = botPoseEstimate.getX();
        double y = botPoseEstimate.getY();
        double z = botPoseEstimate.getZ();
        
        Pose3d botPose = new Pose3d(x,y,z);

        double distance = Math.sqrt(Math.pow(botPose.getX(), 2) + Math.pow(botPose.getY(), 2));

        return distance;
    }
}


// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// public class Distance2AprilTag {
// 	public double getLimeLightDistance() {
// 		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
// 		NetworkTableEntry ty = table.getEntry("ty");
// 		double targetOffsetAngle_Vertical = ty.getDouble(0.0);

// 		// how many degrees back is your limelight rotated from perfectly vertical?
// 		double limelightMountAngleDegrees = 0; 

// 		// distance from the center of the Limelight lens to the floor
// 		double limelightLensHeightInches = 9; 

// 		// distance from the target to the floor
// 		double goalHeightInches = 10.5; 

// 		double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
// 		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

// 		//calculate distance
// 		double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

// 		return distanceFromLimelightToGoalInches;

// 		// Pose3d targetOffset = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
// 		// return Math.sqrt(Math.pow(targetOffset.getX(), 2) + Math.pow(targetOffset.getZ(), 2));
// 	}


   
//     }
    





    