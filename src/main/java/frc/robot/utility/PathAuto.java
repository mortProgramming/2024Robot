//THIS IS A CLASS FOR ME TO TEST WORKING WITH PATHPLANNER AND CHOREO
//USE ABSOLUTELY NOTHING FROM THIS CLASS, I CANNOT GUARANTE IT WILL WORK
package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private static PathPlannerAuto twoPiece;

  public static void init() {
    drivetrain = Drivetrain.getInstance();
    new InstantCommand(() -> drivetrain.zeroGyroscope(0));

    //Configure path to use swerve settings
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
       () ->{return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Red : false;}, //method for checking current alliance. Path flips if alliance is red
    drivetrain);




    NamedCommands.registerCommand("ScoreInAmp", new SequentialCommandGroup(//Bring arm and wrist to score position, eject note, back to rest
      SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
     // new IntakeToVelocity(Intake.AMP_SHOOT_SPEED).withTimeout(.5),
      SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
    ));

    NamedCommands.registerCommand("Intake", new ParallelCommandGroup(//Active intake and bring wrist out
      //new IntakeToVelocity(INTAKE_SPEED).withTimeout(0.5),
      new WristToPosition(WRIST_INTAKE_POSITION).withTimeout(0.5)
    ));

    NamedCommands.registerCommand("StopIntake", new ParallelCommandGroup(//Disable intake and bring wrist in
     // new IntakeToVelocity(0).withTimeout(0.1),
      new WristToPosition(WRIST_REST_POSITION).withTimeout(0.5)
      ));

    NamedCommands.registerCommand("AutoActive", new InstantCommand(() -> {System.out.println("PATH AUTON IS ACTIVE");}));//A simple print command that should run at the the start of any paths we make(NOT AUTOMATIC, MUST DO OURSELVES)

    //build all path-based autons
    twoPiece = new PathPlannerAuto("PathPlanner 2Piece");
  }
  
  public static Command getTwoPiece(){
    return twoPiece;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
