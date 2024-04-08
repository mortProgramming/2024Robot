package frc.robot.commands.Actions.EndEffector;

import static frc.robot.utility.Constants.Intake.AMP_SHOOT_SPEED;

import edu.wpi.first.math.system.plant.struct.DCMotorStruct;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import static frc.robot.utility.Constants.Lights.*;
import edu.wpi.first.wpilibj.Timer;

public class LightsCommand extends Command {
  /** Creates a new IntakeBeamBreak. */
  private Lights lights;
  private Vision vision;
  private int crashCounter;
  private boolean justCrashed;
  private Timer timer;
  private double maxAccel;
  private double lastAccel;
  private double currentAccel;
  private double maxJerk;
  private double currentJerk;
  private Double jerks[] = new Double[JERK_COUNTER_MAX];
  private double averageJerk;
  private double maxAverageJerk;
  private int jerkCounter;

  public LightsCommand() {
    lights = Lights.getInstance();
    vision = Vision.getInstance();
    timer = new Timer();
    crashCounter = 0;
    justCrashed = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    crashCounter = 0;
    justCrashed = false;
    lastAccel = 0;
    currentAccel = 0;
    maxAverageJerk = 0;
    maxJerk = 0;
    maxAccel = 0;
    for (int i = 0; i > JERK_COUNTER_MAX; i++) {
      jerks[i] = 0d;
    }
    // for (int i : jerks) {
    //   jerks[i] = 0d;
    // }
    // jerks[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lastAccel = currentAccel;
    currentAccel = Drivetrain.getInstance().getAccelMag();
    currentJerk = currentAccel - lastAccel;

    // jerkCounter++;
    // if (jerkCounter > (JERK_COUNTER_MAX - 1)) jerkCounter = 0;
    // jerks[jerkCounter] = currentJerk;
    // averageJerk = averager(jerks, JERK_COUNTER_MAX);

    // if(averageJerk > maxAverageJerk) maxAverageJerk = averageJerk;
    if(currentJerk > maxJerk) maxJerk = currentJerk;
    if(currentAccel > maxAccel) maxAccel = currentAccel;
    
    if (currentJerk > (CRASH_DETECTED_G_PER_S / 50) && justCrashed == false) {
      timer.reset();
      timer.start();
      crashCounter++;
      justCrashed = true;
    }

    else if (currentJerk > (CRASH_DETECTED_G_PER_S / 50)) {
      timer.reset();
      timer.start();
      justCrashed = true;
    }

    // if (averageJerk > (CRASH_DETECTED_G_PER_S / 50) && justCrashed == false) {
    //   timer.reset();
    //   timer.start();
    //   crashCounter++;
    //   justCrashed = true;
    // }

    // else if (averageJerk > (CRASH_DETECTED_G_PER_S / 50)) {
    //   timer.reset();
    //   timer.start();
    //   justCrashed = true;
    // }

    else if (timer.get() > CRASH_ANIMATION_TIME) {
      justCrashed = false;
      timer.reset();
      timer.stop();
    }

    if (Intake.hasNote()) {
        lights.setLightsGreen();
        vision.setCamLights(2);  
    }

    else {
      lights.setLightsBlue();
      vision.setCamLights(1);
    }

    if (justCrashed) {
      lights.setLightsGoldBlink();
      // vision.setCamLights(2);
    }

    SmartDashboard.putNumber("Crash Counter", crashCounter);
    SmartDashboard.putNumber("Max Acceleration", maxAccel);
    SmartDashboard.putNumber("Max Jerk", maxJerk);
    SmartDashboard.putNumber("Max Average Jerk", maxAverageJerk);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.setCamLights(1);
    lights.setLightsBlue();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // private double averager(Double[] func, int length) {
  //   double funcAverage = 0;
  //   for (Double val:func) {
  //     funcAverage = funcAverage + val;
  //   }
  //   return (funcAverage / length);
  // }
}
