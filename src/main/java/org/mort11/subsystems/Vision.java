package org.mort11.subsystems;

import org.mort11.mortlib.hardware.camera.NoteCamera;
import org.mort11.mortlib.hardware.camera.TagCamera;

import static org.mort11.mortlib.hardware.camera.TagCameraTypeEnum.*;
import static org.mort11.mortlib.hardware.camera.NoteCameraTypeEnum.*;

import static org.mort11.config.constants.PortConstants.Vision.*;
import static org.mort11.config.constants.FieldConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static Vision vision;

	private NoteCamera noteCamera;
    private TagCamera tagCamera;

	private AprilTagFieldLayout tagLayout;

    private Vision() {
		noteCamera = new NoteCamera(CORALLIMELIGHT, NOTE_CAMERA);
		tagCamera = new TagCamera(LIMELIGHT, TAG_CAMERA);

		// tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        tagLayout = new AprilTagFieldLayout(APRIL_TAGS, FIELD_LENGTH, FIELD_WIDTH);
	}

	public Pose2d getTagPosition(int tagID) {
		return tagLayout.getTagPose(tagID).get().toPose2d();
	}

	public NoteCamera getNoteCamera() {
		return noteCamera;
	}

	public TagCamera getTagCamera() {
		return tagCamera;
	}
    
    public static Vision getInstance() {
		if (vision == null) {
			vision = new Vision();
		}
		return vision;
	}
}
