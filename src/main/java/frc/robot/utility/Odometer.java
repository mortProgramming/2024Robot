package frc.robot.utility;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.Timer;

public class Odometer{

    private static Drivetrain drivetrain;
    private static Vision vision;

    private static Timer timer;
    private static SwerveDriveKinematics driveKinematics;
	private static SwerveDrivePoseEstimator odometry;

    public Odometer() {
        drivetrain = Drivetrain.getInstance();
        vision = Vision.getInstance();

        driveKinematics = drivetrain.getDriveKinematics();

        odometry = new SwerveDrivePoseEstimator(driveKinematics, drivetrain.getNavX().getRotation2d(), drivetrain.getModulePositions(), new Pose2d(new Translation2d(0,0), new Rotation2d(0)));

        timer = new Timer();
        timer.start();
    }

   

    public SwerveDrivePoseEstimator getOdometry () {
        return odometry;
    }

    public double getPoseX () {
        return odometry.getEstimatedPosition().getX();
    }

    public double getPoseY () {
        return odometry.getEstimatedPosition().getY();
    }

  

    /**
     * Resets the odometry pose settings
     * @param visionOverride
     * Determines whether the reset position should be field relative. 
     * If true, new pose will be calculated using limelight megatag systems.
     * If false, new pose will be calculated assuming the robots center is the origin.
     * 
     */

    public void resetOdometry(boolean visionOverride){
        if(visionOverride){
            odometry.resetPosition(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), vision.getFieldPose());
        }
        odometry.resetPosition(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), new Pose2d(0,0,new Rotation2d()));
   }

   /**
    * Resets odometry pose settings
    * @param inputPose
    * Resets the translation and rotation to the given input pose. Mostly used by path-based autonomous routines with a start position.
    */
   public void resetOdometry(Pose2d inputPose){
    odometry.resetPosition(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), inputPose);
   }

   public void resetOdometry(){
    odometry.resetPosition(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), 
    new Pose2d(0,0, new Rotation2d()));
   }
   

   public void updateOdometry () {
        odometry.update(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions());
        
        //checks if Limelight pose measurements are within a certain amount of the ones given by the encoders. If they aren't, the vision measurements are disregarded
        //Pose Comparison will not happen if limelight doesnt have a target
        if(vision.hasTag()){
            if(Math.abs(getPoseX()-vision.getX())<Constants.Vision.MAX_POSE_ERROR_METERS || Math.abs(getPoseY()-vision.getY())<Constants.Vision.MAX_POSE_ERROR_METERS){
                odometry.addVisionMeasurement(vision.getFieldPose(), Timer.getFPGATimestamp());
            }
        }    
        
        SmartDashboard.putNumber("SwervePoseX", getPoseX());
	    SmartDashboard.putNumber("swervePoseY", getPoseY());
        // SmartDashboard.putNumber("swerveTimed", MathSharedStore.getTimestamp());
    }
    public void updateOdometryIgnoreLimelight() {
        odometry.update(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions());    
        
        SmartDashboard.putNumber("SwervePoseX", getPoseX());
	    SmartDashboard.putNumber("swervePoseY", getPoseY());
        // SmartDashboard.putNumber("swerveTimed", MathSharedStore.getTimestamp());
    }
    
    
    public static Odometer getInstance(){
        return new Odometer();
    }
}
