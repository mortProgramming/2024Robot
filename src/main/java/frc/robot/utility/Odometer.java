package frc.robot.utility;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.Timer;

public class Odometer{
    private static boolean canUseLimelight;
    private static Drivetrain drivetrain;
    private static Vision vision;

    private static Timer timer;
    private static SwerveDriveKinematics driveKinematics;
	private static SwerveDrivePoseEstimator odometry;

    private static StructPublisher publisher;
    private static Matrix<N3,N1> poseDeviation = VecBuilder.fill(0.2, 0.1, 0.1);
    private static Matrix<N3,N1> limeDeviation = VecBuilder.fill(0.9,0.9,99999);
    
    
    /*
     * DO NOT USE, ODOMETER IS STATIC
     */
    public Odometer() {
        // drivetrain = Drivetrain.getInstance();
        // vision = Vision.getInstance();

        // driveKinematics = drivetrain.getDriveKinematics();

        // odometry = new SwerveDrivePoseEstimator(driveKinematics, drivetrain.getNavX().getRotation2d(), drivetrain.getModulePositions(), new Pose2d(new Translation2d(0,0), new Rotation2d(0)));

        // timer = new Timer();
        // timer.start();
    }

    public static void OdometerInit() {
        drivetrain = Drivetrain.getInstance();
        vision = Vision.getInstance();

        driveKinematics = drivetrain.getDriveKinematics();

        odometry = new SwerveDrivePoseEstimator(driveKinematics, 
            drivetrain.getNavX().getRotation2d(), 
            drivetrain.getModulePositions(), 
        new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
        poseDeviation,
        limeDeviation
        );

        publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();

        timer = new Timer();
        timer.start();
        
    }

   

    public static SwerveDrivePoseEstimator getOdometry () {
        return odometry;
    }

    public static double getPoseX () {
        return odometry.getEstimatedPosition().getX();
    }

    public static double getPoseY () {
        return odometry.getEstimatedPosition().getY();
    }

    public static Translation2d getPoseTranslation2d() {
        return odometry.getEstimatedPosition().getTranslation();
    }

    public static Rotation2d getPoseRotate() {
        return odometry.getEstimatedPosition().getRotation();
    }

    /**
     * Resets the odometry pose settings
     * @param visionOverride
     * Determines whether the reset position should be field relative. 
     * If true, new pose will be calculated using limelight megatag systems.
     * If false, new pose will be calculated assuming the robots center is the origin.
     * 
     */

    public static void resetOdometry(boolean visionOverride){
        if(visionOverride){
            odometry.resetPosition(drivetrain.getAbsoluteGyroscopeRotation(), drivetrain.getModulePositions(), vision.getFieldPose());
        }
        // odometry.resetPosition(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), new Pose2d(0,0,new Rotation2d()));
   }

   public static void resetOdometry() {
    odometry.resetPosition(drivetrain.getAbsoluteGyroscopeRotation(), drivetrain.getModulePositions(), new Pose2d(0,0,new Rotation2d()));
    }

   /**
    * Resets odometry pose settings
    * @param inputPose
    * Resets the translation and rotation to the given input pose. Mostly used by path-based autonomous routines with a start position.
    */
   public static void resetOdometry(Pose2d inputPose){
    odometry.resetPosition(drivetrain.getAbsoluteGyroscopeRotation(), drivetrain.getModulePositions(), inputPose);
   }
/**
 * OMEGA MUST BE IN RADIANS
 */
   public static void resetOdometry(double x, double y, double omega){
    odometry.resetPosition(drivetrain.getAbsoluteGyroscopeRotation(), drivetrain.getModulePositions(), 
    new Pose2d(x, y, new Rotation2d(omega)));
   }
   

   public static void updateOdometry () {
        odometry.update(drivetrain.getAbsoluteGyroscopeRotation(), drivetrain.getModulePositions());
        
        //checks if Limelight pose measurements are within a certain amount of the ones given by the encoders. If they aren't, the vision measurements are disregarded
        //Pose Comparison will not happen if limelight doesnt have a target
        //Pose comparison will not check angular measurement. We assume the limelight is more accurate in that regard
        //Override will happen if either x or y axis is within max error
        if(vision.hasTag()){
            odometry.addVisionMeasurement(vision.getPose(), Timer.getFPGATimestamp()-vision.getFieldPoseAsArray()[6]);
        }
        
        if(getPoseX() > 16.10 ){
            resetOdometry(16.10, getPoseY(), getPoseRotate().getRadians());
        }
        if(getPoseX() < 0.44){
            resetOdometry(0.44, getPoseY(), getPoseRotate().getRadians());
        }
        if(getPoseY() > 7.85){
            resetOdometry(getPoseX(), 8.36, getPoseRotate().getRadians());
        }
        if(getPoseY()< .37){
            resetOdometry(getPoseX(), .47, getPoseRotate().getRadians());
        }
        
        SmartDashboard.putBoolean("LIMELIGHT POSE IN RANGE", canUseLimelight);
	    SmartDashboard.putNumber("swervePoseY", getPoseY());
        SmartDashboard.putNumber("swervePoseX", getPoseX());

        publisher.set(new Pose2d(new Translation2d(getPoseX(), getPoseY()), getPoseRotate()));
        // SmartDashboard.putNumber("swerveTimed", MathSharedStore.getTimestamp());
    }
}
