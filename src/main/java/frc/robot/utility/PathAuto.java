//THIS IS A CLASS FOR ME TO TEST WORKING WITH PATHPLANNER AND CHOREO
//USE ABSOLUTELY NOTHING FROM THIS CLASS, I CANNOT GUARANTE IT WILL WORK
package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Actions.EndEffector.ArmToPosition;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.SetArmAndWristPos;
import frc.robot.commands.Actions.EndEffector.WristToPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;

import static frc.robot.utility.Constants.Arm.ARM_WRIST_TIMEOUT;
import static frc.robot.utility.Constants.Intake.INTAKE_SPEED;
import static frc.robot.utility.Constants.Wrist.WRIST_INTAKE_POSITION;
import static frc.robot.utility.Constants.Wrist.WRIST_REST_POSITION;
import static frc.robot.utility.Constants.Wrist.WRIST_SCORE_POSITION;

import java.sql.Driver;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.utility.Constants.Drivetrain.AutonConstants;
import frc.robot.utility.Constants.Drivetrain.AutonConstants.*;
import frc.robot.utility.Constants.*;


public class PathAuto extends SubsystemBase {
  private AutoBuilder autoBuilder;
  private static Drivetrain drivetrain;
  private static Odometer odometry = Odometer.getInstance();

  public static void init() {
    drivetrain = Drivetrain.getInstance();

    //GET NUMBERS
    AutoBuilder.configureHolonomic(
    () -> {return odometry.getOdometry().getEstimatedPosition();},  //get current robot position on the field
    (Pose2d startPose) -> {odometry.resetOdometry(startPose);}, //reset odometry to a given pose. WILL ONLY RUN IF AUTON HAS A SET POSE, DOES NOTHING OTHERWISE. Need to compare the Choreo coordinate system to the limelight one
    () -> {return drivetrain.getChassisSpeeds();}, //get the current ROBOT RELATIVE SPEEDS
    (ChassisSpeeds robotRelativeOutput) -> {drivetrain.drive(robotRelativeOutput);}, //makes the robot move given ROBOT RELATIVE CHASSISSPEEDS
    new HolonomicPathFollowerConfig(new PIDConstants(AutonConstants.AUTON_POSITION_KP, AutonConstants.AUTON_POSITION_KI, AutonConstants.AUTON_POSITION_KD), //position PID
        new PIDConstants(AutonConstants.AUTON_ROTATION_KP, AutonConstants.AUTON_ROTATION_KI, AutonConstants.AUTON_ROTATION_KD), //rotation PID
        AutonConstants.MAX_AUTON_VELOCITY, //max Module Speed in M/s
        RobotSpecs.DRIVEBASE_RADIUS_IN_METERS, //todo: Find this number
       new ReplanningConfig()), 
       () ->{return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Red : false;}, 
    drivetrain);
    NamedCommands.registerCommand("ScoreInAmp", new SequentialCommandGroup(
      new SetArmAndWristPos(Arm.ARM_AMP_POSITION, WRIST_SCORE_POSITION).withTimeout(ARM_WRIST_TIMEOUT),
      new IntakeToVelocity(Intake.AMP_SHOOT_SPEED).withTimeout(.5),
      new SetArmAndWristPos(Arm.ARM_REST_POSITION, WRIST_REST_POSITION)
    ));
    NamedCommands.registerCommand("Intake", new ParallelCommandGroup(
      new IntakeToVelocity(INTAKE_SPEED),
      new WristToPosition(WRIST_INTAKE_POSITION)
    ));
    NamedCommands.registerCommand("StopIntake", new ParallelCommandGroup(
      new IntakeToVelocity(0),
      new WristToPosition(WRIST_REST_POSITION)
      ));

  }
  
  public static Command getTwoPiece(){
    return new PathPlannerAuto("PathPlanner 2Piece");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
