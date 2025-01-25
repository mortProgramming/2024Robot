package org.mort11.commands.actions.drivetrain;

import org.mort11.subsystems.Drivetrain;
import org.mort11.subsystems.LimelightHelpers;
import org.mort11.subsystems.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.wpi.first.math.geometry.Pose2d;
import org.mort11.first.math.controller.ProfiledPIDController;
import org.mort11.subsystems.LimelightHelpers;



public class Robot2AprilTag extends Command {

    private Drivetrain drivetrain;
    private Distance2AprilTag distanceCalculator;

    private ProfiledPIDController aprilTagXController;
    private ProfiledPIDController aprilTagYController;
    private ProfiledPIDController aprilTagOmegaController;

    public Robot2AprilTag(){
        drivetrain = Drivetrain.getInstance();
        distanceCalculator = new Robot2AprilTag();

        aprilTagXController = new ProfiledPIDController(0.0, 0.0, 0.0);
        aprilTagYController = new ProfiledPIDController(0.0, 0.0, 0.0);
        aprilTagOmegaController = new ProfiledPIDController(0.0, 0.0, 0.0);

        addRequirements(drivetrain);

    }

    @Override
    public void initialize(){
        aprilTagXController.reset();
        aprilTagYController.reset();
        aprilTagOmegaController.reset();
    }

    

    @Override
    public void execute() {
        //distance to apriltag
        double distance = distanceCalculator.getMegaTagDistance();

        //pose of apriltagform limelight
        Pose2d cameraPose = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").toPose2d();
        double targetX = cameraPose.getX();
        double targetY = cameraPose.getY();

        //robot position
        double targetAngle = Math.atan2(targetX, targetY);

        //robot heading
        double robotHeading = drivetrain.getGyroscopeRotation().getRadians();

        //robot heading/ orientiatoin
        double robotHeading = drivetrain.getGyroscopeRotation().getRadians();

        //rotation speed, align the robot with the tag
        double omegaSpeed = aprilTagOmegaController.calculate(robotHeading, targetAngle);

        double translationX = -aprilTagXController.calculate(targetX, 0);
        double translationY = -aprilTagYController.calculate(targetY, 0);


        drivetrain.drive(new ChassisSpeeds(translationX, translationY, omegaSpeed));

        // //distance to april tag
        // Distance2AprilTag distanceCalculator = new Distance2AprilTag();
        // double distance = distanceCalculator.getMegaTagDistance();

        // //pose from Limelight
        // Pose2d cameraPose = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").toPose2d();
        // double targetX = cameraPose.getX();
        // double targetY = cameraPose.getY();

        // //robot position
        // double targetAngle = Math.atan2(targetY, targetX);  

        // //robots heading 
        // double robotHeading = drivetrain.getGyroscopeRotation().getRadians();

        // //rotation speed
        // double omegaSpeed = targetAngle - robotHeading;

        // if (omegaSpeed > Math.PI) {
        //     omegaSpeed -= 2 * Math.PI;
        // } else if (omegaSpeed < -Math.PI) {
        //     omegaSpeed += 2 * Math.PI;
        // }

        // //speed to move towards the target
        // double driveSpeed = Math.min(distance / 100, 1.0);

        // //translation in X and Y directions
        // double translationX = Math.cos(targetAngle) * driveSpeed;
        // double translationY = Math.sin(targetAngle) * driveSpeed;

        // //moves the robot with translation and rotation
        // drivetrain.setDrive(ChassisSpeeds.fromFieldRelativeSpeeds(translationX, translationY, omegaSpeed, drivetrain.getGyroscopeRotation()));
    }

    @Override
    public boolean isFinished() {
        //apriltag no t visible
        return !LimelightHelpers.hasTarget("limelight") || 
                (aprilTagXController.atSetpoint() && aprilTagYController.atSetpoint() && aprilTagOmegaController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}

// import java.util.function.Supplier;

// import org.mort11.subsystems.Drivetrain;
// import org.mort11.subsystems.LimelightHelpers;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.Command;

// public class Robot2AprilTag extends Command {
//     public Drivetrain drivetrain;

//     private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
//     private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
//     private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

//     private static final Transform3d TAG_TO_GOAL =
//         new Transform3d(
//             new Translation3d(1.5, 0.0, 0.0), //translation from tag to goal
//             new Rotation3d(0.0, 0.0, Math.PI)); //rotation from tag to goal

//     private final Supplier<Pose2d> poseProvider;

//     private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
//     private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
//     private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

//     private Pose3d lastTargetPose;

//     //initialize command with drive & pose provider
//     public Robot2AprilTag() {

//         //limelight gets camera data

//         poseProvider = () -> LimelightHelpers.getCameraPose3d_TargetSpace("limelight").toPose2d();

//         xController.setTolerance(0.2);
//         yController.setTolerance(0.2);
//         omegaController.setTolerance(Units.degreesToRadians(3));
//         omegaController.enableContinuousInput(-Math.PI, Math.PI);

//         drivetrain = Drivetrain.getInstance();

//         addRequirements(drivetrain);
//     }

//     @Override
//     public void initialize() {
//         lastTargetPose = new Pose3d();
//         Pose2d robotPose2d = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").toPose2d();
//         omegaController.reset(robotPose2d.getRotation().getRadians());
//         xController.reset(robotPose2d.getX());
//         yController.reset(robotPose2d.getY());
//     }

//     @Override
//     public void execute() {
//         Pose2d robotPose2d = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").toPose2d();
//         Pose3d robotPose = convertToPose3d(robotPose2d);

//         //limelight data
//         double targetX = LimelightHelpers.getTX("limelight");
//         double targetY = LimelightHelpers.getTY("limelight");
//         double targetArea = LimelightHelpers.getTA("limelight");

//         System.out.println("limelight target x " + (LimelightHelpers.getTX("limelight")));
//         System.out.println("limelight target y " + (LimelightHelpers.getTY("limelight")));
//         System.out.println("limelight target a " + (LimelightHelpers.getTA("limelight")));

//         //target is visible
//         if (targetArea > 0) {
//             //calculates the target's 3D pose from limelight data
//             Transform3d cameraToTarget = new Transform3d(
//                 new Translation3d(targetX, targetY, 0),
//                 new Rotation3d(0, 0, 0)
//             );

//             //robot and camera are aligned, computes the target's pose
//             Pose3d cameraPose = robotPose.transformBy(new Transform3d(new Translation3d(0, 0, 0), new Rotation3d()));
//             Pose3d targetPose = cameraPose.transformBy(cameraToTarget);
//             Pose2d goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

//             // Set PID goals based on the target's pose
//             xController.setGoal(goalPose.getX());
//             yController.setGoal(goalPose.getY());
//             omegaController.setGoal(goalPose.getRotation().getRadians());
//         }

//         if ((lastTargetPose.getX() == 0) && (lastTargetPose.getY() == 0) && (lastTargetPose.getZ() == 0)) {
//             //target not visible, stop the robot
//             drivetrain.setDrive(
//                 ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, robotPose2d.getRotation()));
//         } else {
//             //robot to move to target
//             double xSpeed = xController.calculate(robotPose.getX());
//             if (xController.atGoal()) {
//                 xSpeed = 0;
//             }

//             double ySpeed = yController.calculate(robotPose.getY());
//             if (yController.atGoal()) {
//                 ySpeed = 0;
//             }

//             double omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
//             if (omegaController.atGoal()) {
//                 omegaSpeed = 0;
//             }

//             drivetrain.setDrive(
//                 ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, drivetrain.getGyroscopeRotation()));
//         }

//         lastTargetPose = convertToPose3d(robotPose2d);
//         System.out.println("last target pose x " + (lastTargetPose.getX()));
//         System.out.println("last target pose y " + (lastTargetPose.getY()));
//         System.out.println("last target pose z " + (lastTargetPose.getZ()));
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.setDrive(
//                 ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,0, Rotation2d.fromDegrees(0)));
//     }

//     //convert Pose2d to Pose3d
//     private Pose3d convertToPose3d(Pose2d pose2d) {
//         return new Pose3d(
//             pose2d.getX(),
//             pose2d.getY(),
//             0.0,
//             new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians())
//         );
//     }
// }
