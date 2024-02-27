package frc.robot.subsystems;

import static frc.robot.utility.Constants.Climber.*;
import static frc.robot.utility.Constants.RobotSpecs.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
    
    private static Climber climber;

    //left is main motor
    private CANSparkMax rightClimberMotor;
    private CANSparkMax leftClimberMotor;

    // private CANSparkMax rightClimberServo;
    // private CANSparkMax leftClimberServo;
    private Servo leftServo;
    private Servo rightServo;


    private double rightClimberSpeed;
    private double leftClimberSpeed;

    private double rightServoAngle;
    private double leftServoAngle;


    private double rightSetpoint;
    private double leftSetpoint;

    private ProfiledPIDController rightClimberPositionController;
    private ProfiledPIDController leftClimberPositionController;

    private ArmFeedforward rightClimberPostionFeedForward;
    private ArmFeedforward leftClimberPostionFeedForward;

    private static boolean velocityMode;

    public Climber() {
        rightClimberMotor = new CANSparkMax(MASTER_CLIMBER_MOTOR, MotorType.kBrushless);
        leftClimberMotor = new CANSparkMax(FOLLOW_CLIMBER_MOTOR, MotorType.kBrushless);

        // rightClimberServo = new CANSparkMax(RIGHT_CLIMBER_Servo, MotorType.kBrushed);
        // leftClimberServo = new CANSparkMax(LEFT_CLIMBER_Servo, MotorType.kBrushed);

        leftServo = new Servo(LEFT_CLIMBER_SERVO);
        rightServo = new Servo(RIGHT_CLIMBER_SERVO);


        rightClimberPositionController = new ProfiledPIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D, 
        new Constraints(POSITION_PID_V, POSITION_PID_A));
        leftClimberPositionController = new ProfiledPIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D, 
        new Constraints(POSITION_PID_V, POSITION_PID_A));

        rightClimberPostionFeedForward = new ArmFeedforward(POSITION_FF_S, POSITION_FF_G, POSITION_FF_V, POSITION_FF_A);
        leftClimberPostionFeedForward = new ArmFeedforward(POSITION_FF_S, POSITION_FF_G, POSITION_FF_V, POSITION_FF_A);

        velocityMode = true;
    }

    public void init() {
    //add motor initialization
    }

    /**
     * 
     */
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      if(velocityMode) {
        rightClimberMotor.set(rightClimberSpeed);
        leftClimberMotor.set(leftClimberSpeed);
      }
      else {
        rightClimberMotor.set(0);
        leftClimberMotor.set(0);
      }

    //   rightClimberServo.set(rightServoSpeed);
    //   leftClimberServo.set(leftServoSpeed);

        rightServo.setAngle(rightServoAngle);
        leftServo.setAngle(leftServoAngle);


    //   if (rightServoSpeed == 1) {
    //     SmartDashboard.putBoolean("Servo Position", true);
    //   }
    //   else {
    //     SmartDashboard.putBoolean("Servo Position", false);
    //   }
    }

    /**
     * 
     */
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    
    public void setVelocityMode(boolean isVelocityMode){
        velocityMode = isVelocityMode;
    }

    /**
     * 
     */
    public void setRightClimberVelocity(double rightClimberSpeed){
        this.rightClimberSpeed = rightClimberSpeed;
    }

    public void setLeftClimberVelocity(double leftClimberSpeed){
        this.leftClimberSpeed = leftClimberSpeed;
    }

    public void setRightSetPoint(double rightSetpoint){
        this.rightSetpoint = rightSetpoint;
    }

    public void setLeftSetPoint(double leftSetpoint){
        this.leftSetpoint = leftSetpoint;
    }

    public void setRightServo(double rightServoAngle) {
        this.rightServoAngle = rightServoAngle;
    }

    public void setLeftServo(double leftServoAngle) {
        this.leftServoAngle = leftServoAngle;
    }

    

    public double getRightSetpoint(){
        return rightSetpoint;
    }

    public double getLeftSetpoint(){
        return leftSetpoint;
    }

    public double getRightVelocity() {
        return rightClimberMotor.getEncoder().getVelocity();
    }

    public double getLeftVelocity() {
        return leftClimberMotor.getEncoder().getVelocity();
    }

    public double getRightPosition() {
        return rightClimberMotor.getEncoder().getPosition();
	}

    public double getLeftPosition() {
        return leftClimberMotor.getEncoder().getPosition();
	}

    public boolean nearRightSetpoint(){
        return (Math.abs(rightClimberPostionFeedForward.calculate(getRightPosition(), getRightVelocity()) + 
         rightClimberPositionController.calculate(getRightPosition(), rightSetpoint)) / MAX_VOLTAGE) < CLIMBER_NEAR_SETPOINT_ERROR;
    }

    public boolean nearLeftSetpoint(){
        return (Math.abs(leftClimberPostionFeedForward.calculate(getLeftPosition(), getLeftVelocity()) + 
         leftClimberPositionController.calculate(getLeftPosition(), leftSetpoint)) / MAX_VOLTAGE) < CLIMBER_NEAR_SETPOINT_ERROR;
    }

   public void setRightSetpoint(double rightSetpoint){
        this.rightSetpoint = rightSetpoint;
    }

    public void setLeftSetpoint(double leftSetpoint){
        this.leftSetpoint = leftSetpoint;
    }

    /**
     * 
     * @return
     */
    public static Climber getInstance(){
        if (climber==null){
            climber = new Climber();
        }
        return climber;
    }
}
