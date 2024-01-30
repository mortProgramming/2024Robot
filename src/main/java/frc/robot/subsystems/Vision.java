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
	 * @return
	 */
    public boolean hasTarget() {
		return llTable.getEntry("tv").getInteger(0) == 1;
	}

	/**
	 * 
	 * @return
	 */
    public double getTargetArea() {
		return llTable.getEntry("ta").getDouble(0);
	}

	/**
	 * 
	 * @param id
	 */
	public void setPipeline(int id) {
		llTable.getEntry("pipeline").setNumber(id);
	}

	/**
	 * 
	 * @param pipe
	 */
	public void setPipeline(Pipeline pipe) {
		setPipeline(pipe.getId());
	}

	/**
	 * 
	 * @param pipe
	 * @param AprilTagID
	 */
	public void setPipeline(Pipeline pipe, int AprilTagID) {
		setPipeline(pipe.getId(AprilTagID));
	}

	/**
	 * 
	 * @return
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
	 * 
	 * @return
	 */
	public double getX() {
		return llTable.getEntry("tx").getDouble(0);
	}

	/**
	 * 
	 * @return
	 */
	public double getY() {
		return llTable.getEntry("ty").getDouble(0);
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
	 * 
	 * @return
	 */
	public int getAprilTagId() {
		return (int) llTable.getEntry("tid").getInteger(0);
	}

	public double getDistanceToTag(double tagHeight) {
		if(llTable.getEntry("tv").getDouble(0) == 1){
			return (tagHeight - CAMERA_HEIGHT) / (Math.tan(Math.toRadians(getY())));
		}
		return -1;
		
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
