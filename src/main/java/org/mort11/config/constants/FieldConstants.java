package org.mort11.config.constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldConstants {
    public static final double FIELD_LENGTH = 3; //16.541
    public static final double FIELD_WIDTH = 3; //8.211

    public static final List<AprilTag> APRIL_TAGS = List.of(
        new AprilTag(
            1, 
            new Pose3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0)
            )
        )
    );
}
