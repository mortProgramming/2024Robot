package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.RobotSpecs.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants.RobotSpecs;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Arm extends SubsystemBase {
    
    private static boolean velocityMode;

    private static Arm arm;

    //left is main motor
    private static TalonFX masterArmMotor;
    private static TalonFX followArmMotor;

    private CANSparkMax blowerMotor;

    private double armSpeed;
    private double setpoint;

    private double targetBlowerOutput;
    private double currentBlowerOutput;

    private ProfiledPIDController armPositionController;

    private PIDController blowerController;

        // private PIDController armPositionController;

    // private SimpleMotorFeedforward armPostionFeedForward;
        private ArmFeedforward armPostionFeedForward;

        private DutyCycleEncoder encoder;

    public Arm() {
        velocityMode = true;
        targetBlowerOutput = 0;
        currentBlowerOutput = 0;

        masterArmMotor = new TalonFX(MASTER_ARM_MOTOR);
        followArmMotor = new TalonFX(FOLLOW_ARM_MOTOR);

        followArmMotor.setControl(new Follower(MASTER_ARM_MOTOR, false));

        // masterArmMotor.set

        armPositionController = new ProfiledPIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D, 
        new Constraints(POSITION_PID_V, POSITION_PID_A));

        // armPositionController = new PIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D);

        // armPostionFeedForward = new SimpleMotorFeedforward(POSITION_FF_S, POSITION_FF_V, POSITION_FF_A);
        armPostionFeedForward = new ArmFeedforward(POSITION_FF_S, POSITION_FF_G, POSITION_FF_V, POSITION_FF_A);

        encoder = new DutyCycleEncoder(ENCODER_PORT);

        blowerController = new PIDController(BLOWER_PID_P, BLOWER_PID_I, BLOWER_PID_D);
        blowerMotor = new CANSparkMax(BLOWER_MOTOR, MotorType.kBrushless);

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
        // masterArmMotor.set(armSpeed);
        SmartDashboard.putNumber("Arm Postion", getPosition());
        SmartDashboard.putNumber("Arm Position Degrees", posToDegrees());
        SmartDashboard.putNumber("Encoder Arm Postion", getEncoder().getAbsolutePosition());
        SmartDashboard.putNumber("Encoder Arm Position Degrees", encoderToDegrees());
        SmartDashboard.putNumber("Arm Setpoint", setpoint);
        SmartDashboard.putNumber("arm output", setPosition(setpoint));
        SmartDashboard.putNumber("ActualArmMotorOutput", masterArmMotor.get());
        SmartDashboard.putBoolean("isvelocityMode", velocityMode);

        if(velocityMode == true) {
            masterArmMotor.set(-armSpeed);
        }

        else {
            masterArmMotor.set(-setPosition(setpoint));
            // masterArmMotor.set(0);
        }
        //masterArmMotor.set(0);
        // setPosition(setpoint);

        currentBlowerOutput += blowerController.calculate(currentBlowerOutput, targetBlowerOutput);

        blowerMotor.set(currentBlowerOutput);

    }

    /**
     * 
     */
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public void setBlowerTarget(double targetBlowerOutput){
        this.targetBlowerOutput = targetBlowerOutput;
    }

    /**
     * 
     */
    public void setArmVelocity(double armSpeed){
        this.armSpeed = armSpeed;
    }

    public void setArmVelocityG(double armSpeed){
        this.armSpeed = armSpeed + POSITION_FF_G * Math.cos(Math.toRadians(encoderToDegrees()));
    }

    // public void setArmVelocityArmFeed(double armSpeed){
    //     this.armSpeed = armSpeed + POSITION_FF_G * Math.sin(Math.toRadians(posToDegrees())) + armPostionFeedForward.calculate(getPosition(), getVelocity());
    // }

    // public void setArmVelocityArmFeed(double armSpeed){
    //     this.armSpeed = armSpeed + armPostionFeedForward.calculate(getPosition(), getVelocity());
    // }

    public void setSetPoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void setVoltage(double voltage){
        masterArmMotor.setVoltage(voltage);
    }

    public double getSetpoint(){
        return setpoint;
    }

    public double getVelocity() {
        return masterArmMotor.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
		return masterArmMotor.getPosition().getValueAsDouble();
	}

    // encoder
    public double getEncoderPosition() {
		return encoder.getAbsolutePosition();
	}

    public double posToDegrees(){
        return (getPosition() * ARM_GEAR_RATIO)  +  ARM_DEGREES_TO_0;
    }

    //encoder
    public double encoderToDegrees() {
        double degrees = getEncoderPosition() * 360 + ARM_ENCODER_DEGREES_TO_0;
        if (degrees < 0) {
            degrees += 360;
        }

        if (degrees > ARM_NEVER_POSITION) {
            degrees -= 360;
        }

        if (degrees < -90 && degrees > -270) {
            degrees += 360;
        }

        // degrees = degrees;

        return degrees;
        // if(getEncoderPosition() < 0) {
        //     return ((1 - getEncoderPosition()) * 360) + ARM_ENCODER_DEGREES_TO_0;
        // }
        // else {
        //     return(getEncoderPosition() * 360) + ARM_ENCODER_DEGREES_TO_0;

        // }    
    }

    // public boolean nearSetpoint(){
    //     return (Math.abs(armPostionFeedForward.calculate(getPosition(), getVelocity()) + 
    //      armPositionController.calculate(getPosition(),setpoint)) / MAX_VOLTAGE) < ARM_NEAR_SETPOINT_ERROR;
    // }
    public boolean nearSetpoint(){
        return encoderToDegrees()+ARM_NEAR_SETPOINT_ERROR >= setpoint && encoderToDegrees()-ARM_NEAR_SETPOINT_ERROR <= setpoint;
    }

    /**
     * 
     * 
     */
    public static Arm getInstance(){
        if (arm==null){
            arm = new Arm();

        }
        return arm;
    }

    public TalonFX getArmMaster(){
        return masterArmMotor;
    }

    public void setVelocityMode(boolean isVelocityMode){
        velocityMode = isVelocityMode;
    }

    public double getVoltage() {
        // TODO Auto-generated method stub
        return getArmMaster().getMotorVoltage().getValueAsDouble();
    }

    public DutyCycleEncoder getEncoder() {
        return encoder;
    }

    private double setPosition(double setpoint) {
		double output = (POSITION_FF_G * Math.cos(Math.toRadians(encoderToDegrees())))
        - armPositionController.calculate(encoderToDegrees(), setpoint);
		// if (output >= 1){
		// 	output = 0.1;
		// } else if (output <= -1) {
		// 	output = -0.1;
		// }
		// double output = 0.1 * sin(getPositionDegrees()) +
		// armController.calculate(getPosition(), setpoint);
		// masterArmMotor.set(output);

        return output;
	}
}
