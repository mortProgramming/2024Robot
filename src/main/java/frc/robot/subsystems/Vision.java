package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants.Vision.Pipeline;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.utility.Constants.Vision.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class Vision extends SubsystemBase {
    public static Vision vision;

    	private NetworkTable tagTable;
		private NetworkTable intakeTable;
	
		private double totalOutlier;
		private double newValue;
		private double position;
		private double totalPosition;
		private int valuePosition;
		private double averageOutlier;
		private double averagePosition;
		private double outlierCounter;
		private double noOutlierCounter;
		private double nonOutlierCounter;
		private double[] values = new double[AMOUNT_TEST_FRAMES];
		private double[] outliers = new double[MAX_OUTLIERS];
		private VideoSource intakeCam;
		
    public Vision() {
        // tagTable = NetworkTableInstance.getDefault().getTable("limelight");
		tagTable = NetworkTableInstance.getDefault().getTable("TagLight");
		intakeTable = NetworkTableInstance.getDefault().getTable("intakeLight");
		
    }

	/**
	 * 
	 * @return True if the limelight has a target, false if no target
	 */
    public boolean hasTag() {
		return tagTable.getEntry("tv").getInteger(0) == 1;
	}
	/**
	 * 
	 * @return True if the limelight has a target, false if no target
	 */
    public boolean hasNote() {
		return intakeTable.getEntry("tv").getInteger(0) == 1;
	}

	public double getNoteXDegrees() {
		return intakeTable.getEntry("tx").getDouble(0);
	}

	/**
	 * 
	 * @return The area of the targeted apriltag, if any. -1 if no target found
	 */
    public double getTagArea() {
		if(hasTag()){
			return tagTable.getEntry("ta").getDouble(0);
		}
		return -1;
		
	}
	/**
	 * 
	 * @return The area of the targeted note piece, if any. -1 if no target found
	 */
	public double getNoteArea() {
		if(hasNote()){
			return tagTable.getEntry("ta").getDouble(0);
		}
		return -1;
		
	}




	// Translation (x, y, z) Rotation(pitch, yaw, roll)
	/**
	 * 
	 * @return
	 */
	public Number[] getCamTranslation() {
		return tagTable.getEntry("targetpose_robotspace").getNumberArray(new Number[0]);
	}

	/**
	 * Get the X value of the target in degrees
	 * @return The X value of the target in degrees, -1 if no target found.
	 */
	public double getX() {
		if(hasTag()){
			return tagTable.getEntry("tx").getDouble(0);
		}
		return -1;
	}

	/**
	 * Get the Y value of the target in degrees
	 * @return The Y value of the target in degrees, -1 if no target found
	 */
	public double getY() {
		if(hasTag()){
			return tagTable.getEntry("ty").getDouble(0);
		}
		return -1;
	}

	public double getCamTranslationX() {
		if (getCamTranslation().length < 1) {
			return 0;
		}

		return (double) getCamTranslation()[0];
	}

	public double getCamTranslationY() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[1];
	}

	public double getCamTranslationZ() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[2];
	}


	public double getCamTranslationPitch() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[3];
	}

	
	public double getCamTranslationYaw() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[4];
	}

	
	public double getCamTranslationRoll() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[5];
	}

	/**
	 * 
	 * @return
	 */
	public Pose2d getPose() {
		double[] poseNum = new double[6];

		// if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
		// 	poseNum = tagTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
		// } else {
		// 	poseNum = tagTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		// }

		return new Pose2d(poseNum[0], poseNum[1], new Rotation2d(Math.toRadians(poseNum[5])));
	}

	/**
	 * 
	 * @return
	 */
	public double getLatency() {
		return tagTable.getEntry("botpose").getDoubleArray(new double[6])[6];
	}

	/**
	 * Get the ID of any currently targeted AprilTag
	 * @return ID of targeted apriltag as an int, -1 if no target found
	 */
	public int getAprilTagId() {
		if(hasTag()){
			return (int) tagTable.getEntry("tid").getInteger(0);
		}
		return -1;
		
	}


	// old, prior to 3d vision
	/**
	 * Gets the distance to the target AprilTag in inches
	 * @param tagHeight 
	 * The Height of the tag. All tag heights are in constants
	 * @return The Height of the tag in inches. If no target, return -1
	 */
	// public double getDistanceToTag(double tagHeight) {
	// 	if(tagTable.getEntry("tv").getDouble(0) == 1){
	// 		return (tagHeight - CAMERA_HEIGHT) / (Math.tan(Math.toRadians(getY())));
	// 	}
	// 	return -1;
		
	// }
	/*
	 * Gets the robots pose on the field relative to the driverstation.
	 */
	public Pose2d getFieldPose() {
		double[] poseNum = new double[6];
		
		if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
			poseNum = tagTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
		} else {
			poseNum = tagTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		}

		return new Pose2d(poseNum[0], poseNum[1], new Rotation2d(Math.toRadians(poseNum[5])));
	}

	/**
     * 
     */
    @Override
    public void periodic() {
		// totalOutlier = 0; 
		// newValue = position;
		// totalPosition = 0;
		// valuePosition++;

		// if(valuePosition == AMOUNT_TEST_FRAMES){
		// 	valuePosition = 0;
		// }
		
		// for(int counter = 0; counter < MAX_OUTLIERS; counter++){
		// 	totalPosition += totalPosition + values[counter];
		// }

		// averagePosition = totalPosition/AMOUNT_TEST_FRAMES;
		
		// if(outlierCounter > MAX_OUTLIERS){
		// 	for(int counter = 0; counter < MAX_OUTLIERS; counter++){
		// 		totalOutlier += outliers[counter];
		// 	}
		// 	averageOutlier = totalOutlier/AMOUNT_TEST_FRAMES;
		// 	for(int counter = 0; counter < MAX_OUTLIERS; counter++){
		// 		values[counter] = averageOutlier;
		// 	}
		// 	outlierCounter = 0;
		// 	noOutlierCounter = 0;
		// }else if(newValue < averagePosition + MAX_ERROR && newValue > (averagePosition - MAX_ERROR)){
		// 	values[valuePosition] = newValue;
		// 	nonOutlierCounter++;
		// 	if(nonOutlierCounter > MAX_NON_OUTLIERS){
		// 		nonOutlierCounter = 0;
		// 		outlierCounter = 0;
		// 	}
		// }else{
		// 	nonOutlierCounter = 0;
		// 	outliers[(int) outlierCounter] = newValue;
		// 	outlierCounter++;
		// }
		SmartDashboard.putNumber("note x", getNoteXDegrees());
    }

	/**
	 * 
	 */
	
	public static Vision getInstance() {
		if (vision == null) {
			vision = new Vision();
		}
		return vision;
	}
}
