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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Actions.RobotStart;
import frc.robot.commands.Actions.EndEffector.IntakeBeamBreak;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.SpitNote;
import frc.robot.commands.Actions.EndEffector.ArmWrist.ArmToPosition;
import frc.robot.commands.Actions.EndEffector.ArmWrist.SetArmAndWristPos;
import frc.robot.commands.Actions.EndEffector.ArmWrist.WristToPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;

import static frc.robot.utility.Constants.Arm.ARM_WRIST_TIMEOUT;
import static frc.robot.utility.Constants.Intake.AUTO_SHOOT_SPEED;
import static frc.robot.utility.Constants.Intake.INTAKE_SPEED;
import static frc.robot.utility.Constants.Intake.SHOOTER_SHOOT_SPEED;
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
  private static PathPlannerAuto twoPiece;
  private static PathPlannerAuto gackleyAuto;
  private static PathPlannerAuto bieryAuto;
  private static PathPlannerAuto pureTwoPiece;
  private static PathPlannerAuto gackleyAutoPure;
  private static PathPlannerAuto choreoOneNote;
  private static Command twoPieceAmpSide;

  public static void init() {
    drivetrain = Drivetrain.getInstance();
    new InstantCommand(() -> drivetrain.zeroGyroscope(0));

    //Configure path to use swerve settings
    AutoBuilder.configureHolonomic(
    () -> {return Odometer.getOdometry().getEstimatedPosition();},  //get current robot position on the field
    (Pose2d startPose) -> {Odometer.resetOdometry(startPose);}, //reset odometry to a given pose. WILL ONLY RUN IF AUTON HAS A SET POSE, DOES NOTHING OTHERWISE. 
    () -> {return drivetrain.getChassisSpeeds();}, //get the current ROBOT RELATIVE SPEEDS
    (ChassisSpeeds robotRelativeOutput) -> {drivetrain.drive(robotRelativeOutput);}, //makes the robot move given ROBOT RELATIVE CHASSISSPEEDS
    new HolonomicPathFollowerConfig(
      new PIDConstants(AutonConstants.AUTON_POSITION_KP, AutonConstants.AUTON_POSITION_KI, AutonConstants.AUTON_POSITION_KD), //position PID
        new PIDConstants(AutonConstants.AUTON_ROTATION_KP, AutonConstants.AUTON_ROTATION_KI, AutonConstants.AUTON_ROTATION_KD), //rotation PID
        AutonConstants.MAX_AUTON_VELOCITY, //max Module Speed in M/s
        RobotSpecs.DRIVEBASE_RADIUS_IN_METERS, //todo: Find this number
       new ReplanningConfig()), 
       () ->{return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Red : false;}, //method for checking current alliance. Path flips if alliance is red
    drivetrain);

    NamedCommands.registerCommand("ScoreInAmp", new SequentialCommandGroup(//Bring arm and wrist to score position, eject note, back to rest
      new WristToPosition(WRIST_REST_POSITION).withTimeout(.01),
      SetArmAndWristPos.score().withTimeout(ARM_WRIST_TIMEOUT),
      new IntakeToVelocity(AUTO_SHOOT_SPEED).withTimeout(.4),
      SetArmAndWristPos.rest().withTimeout(ARM_WRIST_TIMEOUT)
    ).withTimeout(3.45));

    NamedCommands.registerCommand("Intake", new ParallelCommandGroup(//Active intake and bring wrist out
    new PrintCommand("RUNNING INTAKE"),  
    new IntakeBeamBreak(WRIST_REST_POSITION)
    ).withTimeout(2.2));
    
    NamedCommands.registerCommand("IntakeStayOut", new ParallelCommandGroup(//Active intake and bring wrist out
      new IntakeBeamBreak(WRIST_INTAKE_POSITION)
    ));

    NamedCommands.registerCommand("Spit", new ParallelCommandGroup(//Disable intake and bring wrist in
      new SpitNote()
      ));

    NamedCommands.registerCommand("AutoActive", new SequentialCommandGroup(new InstantCommand(() -> {System.out.println("PATH AUTON IS ACTIVE");})));//A simple print command that should run at the the start of any paths we make(NOT AUTOMATIC, MUST DO OURSELVES)
    NamedCommands.registerCommand("FieldOrient", new RobotStart((DriverStation.getAlliance().get() == Alliance.Red) ?  90 : 270));
    NamedCommands.registerCommand("Outtake", new IntakeToVelocity(-0.65).withTimeout(.75));
    //build all path-based autons
    twoPiece = new PathPlannerAuto("PathPlanner2PieceTest");
    gackleyAuto = new PathPlannerAuto("GackleyAuto1");
    bieryAuto = new PathPlannerAuto("BieryWildAuto");
    choreoOneNote = new PathPlannerAuto("OneNote");
    twoPieceAmpSide = new PathPlannerAuto("TwoPieceAmpSide");
  }
  
  public static Command getTwoPiece(){
    return twoPiece;
  }
  public static Command getPurePathTwoPiece(){
    return pureTwoPiece;
  }
  public static Command getGackleyAuto(){
    return gackleyAuto;
  }
  public static Command getBieryAuto(){
    return bieryAuto;
  }
  public static Command getGackleyAutoPure(){
    return gackleyAutoPure;
  }
  public static Command getChoreoOneNote(){
    return choreoOneNote;
  }
  public static Command getTwoPieceAmpSide(){
    return twoPieceAmpSide;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
