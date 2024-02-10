package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.Vision.Pipeline;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static frc.robot.util.Constants.Vision.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class Vision extends SubsystemBase {
    public static Vision vision;

    	private NetworkTable llTable;
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
		
    public Vision() {
        llTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

	/**
	 * 
	 * @return True if the limelight has a target, false if no target
	 */
    public boolean hasTarget() {
		return llTable.getEntry("tv").getInteger(0) == 1;
	}

	/**
	 * 
	 * @return The area of the target, if any. -1 if no target found
	 */
    public double getTargetArea() {
		if(hasTarget()){
			return llTable.getEntry("ta").getDouble(0);
		}
		return -1;
		
	}

	/**
	 * Switch limelight pipeline based on a given number to switch to
	 * @param id
	 * The pipeline ID to switch to
	 */
	public void setPipeline(int id) {
		llTable.getEntry("pipeline").setNumber(id);
	}

	/**
	 * Set the current pipeline based on the entered Pipeline object
	 * @param pipe
	 * A pipeline object to put in. The ID to switch to is obtained from this object
	 */
	public void setPipeline(Pipeline pipe) {
		setPipeline(pipe.getId());
	}

	/**
	 * Switch the pipeline to whatever tracks the target apriltag
	 * @param pipe
	 * The pipeline class
	 * @param AprilTagID
	 * The apriltag you are looking for
	 */
	public void setPipeline(Pipeline pipe, int AprilTagID) {
		setPipeline(pipe.getId(AprilTagID));
	}

	/**
	 * Gets the currently used pipeline
	 * @return the Pipeline object currently being used
	 */
	public Pipeline getPipelineID() {
		return Pipeline.getPipeline((int) llTable.getEntry("pipeline").getInteger(0));
	}

	// Translation (x, y, z) Rotation(pitch, yaw, roll)
	/**
	 * 
	 * @return
	 */
	public Number[] getCamTranslation() {
		return llTable.getEntry("targetpose_robotspace").getNumberArray(new Number[0]);
	}

	/**
	 * Get the X value of the target in degrees
	 * @return The X value of the target in degrees, -1 if no target found.
	 */
	public double getX() {
		if(hasTarget()){
			return llTable.getEntry("tx").getDouble(0);
		}
		return -1;
	}

	/**
	 * Get the Y value of the target in degrees
	 * @return The Y value of the target in degrees, -1 if no target found
	 */
	public double getY() {
		if(hasTarget()){
			return llTable.getEntry("ty").getDouble(0);
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

	/**
	 * 
	 * @return
	 */
	public double getCamTranslationPitch() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[3];
	}

	/**
	 * 
	 * @return
	 */
	public double getCamTranslationYaw() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[4];
	}

	/**
	 * 
	 * @return
	 */
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
		// 	poseNum = llTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
		// } else {
		// 	poseNum = llTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		// }

		return new Pose2d(poseNum[0], poseNum[1], new Rotation2d(Math.toRadians(poseNum[5])));

	}

	/**
	 * 
	 * @return
	 */
	public double getLatency() {
		return llTable.getEntry("botpose").getDoubleArray(new double[6])[6];
	}

	/**
	 * Get the ID of any currently targeted AprilTag
	 * @return ID of targeted apriltag as an int, -1 if no target found
	 */
	public int getAprilTagId() {
		if(hasTarget()){
			return (int) llTable.getEntry("tid").getInteger(0);
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
	// 	if(llTable.getEntry("tv").getDouble(0) == 1){
	// 		return (tagHeight - CAMERA_HEIGHT) / (Math.tan(Math.toRadians(getY())));
	// 	}
	// 	return -1;
		
	// }

	public Pose2d getFieldPose() {
		double[] poseNum = new double[6];
		
		if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
			poseNum = llTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
		} else {
			poseNum = llTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		}

		return new Pose2d(poseNum[0], poseNum[1], new Rotation2d(Math.toRadians(poseNum[5])));
	}

	/**
     * 
     */
    @Override
    public void periodic() {
		totalOutlier = 0; 
		newValue = position;
		totalPosition = 0;
		valuePosition++;

		if(valuePosition == AMOUNT_TEST_FRAMES){
			valuePosition = 0;
		}
		
		for(int counter = 0; counter < MAX_OUTLIERS; counter++){
			totalPosition += totalPosition + values[counter];
		}

		averagePosition = totalPosition/AMOUNT_TEST_FRAMES;
		
		if(outlierCounter > MAX_OUTLIERS){
			for(int counter = 0; counter < MAX_OUTLIERS; counter++){
				totalOutlier += outliers[counter];
			}
			averageOutlier = totalOutlier/AMOUNT_TEST_FRAMES;
			for(int counter = 0; counter < MAX_OUTLIERS; counter++){
				values[counter] = averageOutlier;
			}
			outlierCounter = 0;
			noOutlierCounter = 0;
		}else if(newValue < averagePosition + MAX_ERROR && newValue > (averagePosition - MAX_ERROR)){
			values[valuePosition] = newValue;
			nonOutlierCounter++;
			if(nonOutlierCounter > MAX_NON_OUTLIERS){
				nonOutlierCounter = 0;
				outlierCounter = 0;
			}
		}else{
			nonOutlierCounter = 0;
			outliers[(int) outlierCounter] = newValue;
			outlierCounter++;
		}
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
