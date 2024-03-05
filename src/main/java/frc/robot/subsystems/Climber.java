package frc.robot.subsystems;

import static frc.robot.utility.Constants.Climber.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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


    private Servo rightServo;
    private Servo leftServo;


    private double rightClimberSpeed;
    private double leftClimberSpeed;

    private double rightServoAngle;
    private double leftServoAngle;


    private double rightSetpoint;
    private double leftSetpoint;

    private ProfiledPIDController rightClimberPositionController;
    private ProfiledPIDController leftClimberPositionController;
 
    private double tolerance = 4;

    private static boolean velocityMode;

    public Climber() {
        rightClimberMotor = new CANSparkMax(MASTER_CLIMBER_MOTOR, MotorType.kBrushless);
        leftClimberMotor = new CANSparkMax(FOLLOW_CLIMBER_MOTOR, MotorType.kBrushless);

       leftServo = new Servo(LEFT_CLIMBER_SERVO);
       rightServo = new Servo(RIGHT_CLIMBER_SERVO);

       leftServoAngle = 90;
       rightServoAngle = 90;


        rightClimberPositionController = new ProfiledPIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D, 
        new Constraints(POSITION_PID_V, POSITION_PID_A));
        leftClimberPositionController = new ProfiledPIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D, 
        new Constraints(POSITION_PID_V, POSITION_PID_A));
        
        rightClimberPositionController.setTolerance(tolerance);
        leftClimberPositionController.setTolerance(tolerance);

        velocityMode = true;
        
        
    }

    public ProfiledPIDController getLeftController(){
        return leftClimberPositionController;
    }
    public ProfiledPIDController getRightController(){
        return rightClimberPositionController;
    }
    public Servo getLeftServo(){
        return leftServo;
    }
    public Servo getRightServo(){
        return rightServo;
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
        leftClimberMotor.set(leftClimberPositionController.calculate(leftClimberMotor.getEncoder().getPosition(), leftSetpoint));
        rightClimberMotor.set(rightClimberPositionController.calculate(rightClimberMotor.getEncoder().getPosition(), rightSetpoint));
    }
      SmartDashboard.putNumber("LeftClimberEncoder", leftClimberMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("RightClimberEncoder", rightClimberMotor.getEncoder().getPosition());

    rightServo.setAngle(rightServoAngle);
    leftServo.setAngle(leftServoAngle);


    if (rightServoAngle == 90) {
        SmartDashboard.putBoolean("Climber Locked", true);
    }

    else {
        SmartDashboard.putBoolean("Climber Locked", false);
    }


    SmartDashboard.putBoolean("Climber Velocity Mode ON/OFF", velocityMode);
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


    public CANSparkMax getRightClimberMotor(){
        return this.rightClimberMotor;
     }

     public CANSparkMax getLeftClimberMotor(){
        return this.leftClimberMotor;
     }

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
        return rightClimberPositionController.atSetpoint();
    }

    public boolean nearLeftSetpoint(){
        return leftClimberPositionController.atSetpoint();
    }

    public void setRightSetpoint(double rightSetpoint){
        this.rightSetpoint = rightSetpoint;
    }

    public void setLeftSetpoint(double leftSetpoint){
        this.leftSetpoint = leftSetpoint;
    }

    
    public static Climber getInstance(){
        if (climber==null){
            climber = new Climber();
        }
        return climber;
    }
}
