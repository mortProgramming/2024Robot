package org.mort11.subsystems;

import static org.mort11.configuration.constants.PhysicalConstants.Climber.*;
import static org.mort11.configuration.constants.PIDConstants.Climber.*;
import static org.mort11.configuration.constants.PortConstants.Climber.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
    
    private static Climber climber;

    //left is main motor
    private CANSparkMax rightClimberMotor;
    private CANSparkMax leftClimberMotor;
    
    private Servo leftServo;
    private Servo rightServo;

    private double rightClimberSpeed;
    private double leftClimberSpeed;

    private double rightServoAngle;
    private double leftServoAngle;

    private ProfiledPIDController rightClimberPositionController;
    private ProfiledPIDController leftClimberPositionController;

    public Climber() {
        rightClimberMotor = new CANSparkMax(MASTER_CLIMBER_MOTOR, MotorType.kBrushless);
        leftClimberMotor = new CANSparkMax(FOLLOW_CLIMBER_MOTOR, MotorType.kBrushless);
        leftServo = new Servo(LEFT_CLIMBER_SERVO);
        rightServo = new Servo(RIGHT_CLIMBER_SERVO);

        leftServoAngle = SERVO_GLOBAL_LOCK_POS;
        rightServoAngle = SERVO_GLOBAL_LOCK_POS;

        rightClimberPositionController = new ProfiledPIDController(
            POS_KP, POS_KI, POS_KD, POS_CONSTRAINTS
        );
        leftClimberPositionController = new ProfiledPIDController(
            POS_KP, POS_KI, POS_KD, POS_CONSTRAINTS
        );
        
        rightClimberPositionController.setTolerance(POS_POS_TOLERANCE);
        leftClimberPositionController.setTolerance(POS_POS_TOLERANCE);
    }

    @Override
    public void periodic() {
        rightClimberMotor.set(rightClimberSpeed);
        leftClimberMotor.set(leftClimberSpeed);
        rightServo.setAngle(rightServoAngle);
        leftServo.setAngle(leftServoAngle);

        SmartDashboard.putNumber("LeftClimberEncoder", leftClimberMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("RightClimberEncoder", rightClimberMotor.getEncoder().getPosition());
        
        //returns greeen when servo locked
        SmartDashboard.putBoolean("Climber Locked", rightServoAngle == SERVO_GLOBAL_LOCK_POS);
    }

    public void setLeftVelocity(double leftClimberSpeed) {
        this.leftClimberSpeed = leftClimberSpeed;
    }

    public void setRightVelocity(double rightClimberSpeed) {
        this.rightClimberSpeed = rightClimberSpeed;
    }

    public void setLeftSetpoint(double leftSetpoint) {
        leftClimberSpeed = leftClimberPositionController.calculate(
            leftClimberMotor.getEncoder().getPosition(), leftSetpoint
        );
    }

    public void setRightSetpoint(double rightSetpoint) {
        rightClimberSpeed = rightClimberPositionController.calculate(
            rightClimberMotor.getEncoder().getPosition(), rightSetpoint
        );
    }

    public void setLeftServo(double leftServoAngle) {
        this.leftServoAngle = leftServoAngle;
    }

    public void setRightServo(double rightServoAngle) {
        this.rightServoAngle = rightServoAngle;
    }

    

    public boolean getLeftSetpoint() {
        return leftClimberPositionController.atSetpoint();
    }

    public boolean getRightSetpoint() {
        return rightClimberPositionController.atSetpoint();
    }

    public static Climber getInstance() {
        if (climber == null){
            climber = new Climber();
        }
        return climber;
    }
}
