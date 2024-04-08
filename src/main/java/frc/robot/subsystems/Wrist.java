package frc.robot.subsystems;

import static frc.robot.utility.Constants.RobotSpecs.*;
import static frc.robot.utility.Constants.Wrist.*;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends SubsystemBase {
    
    private static Wrist wrist;

    private TalonFX wristMotor;
    private double wristSpeed;

    private double setpoint;

    private boolean velocityMode;

    private ProfiledPIDController wristPositionController;
    private SimpleMotorFeedforward wristPostionFeedForward;
    private Servo trapServo;
    private double servoPos;


    public Wrist() {
        velocityMode = true;
        wristMotor = new TalonFX(WRIST_MOTOR);
    
        wristPositionController = new ProfiledPIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D, 
        new Constraints(POSITION_PID_V, POSITION_PID_A));

        wristPostionFeedForward = new SimpleMotorFeedforward(POSITION_FF_S, POSITION_FF_V, POSITION_FF_A);
        trapServo = new Servo(TRAP_SERVO_PORT);
        servoPos = 90;
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
        wristMotor.set(wristSpeed);
        SmartDashboard.putNumber("Wrist Position", getPosition());
        SmartDashboard.putNumber("Wrist Position Degrees", posToDegrees());
        SmartDashboard.putNumber("Wrist Setpoint", setpoint);
        SmartDashboard.putNumber("Wrist output", setPosition(setpoint));
        SmartDashboard.putNumber("ActualWristMotorOutput", wristMotor.get());
        SmartDashboard.putBoolean("isvelocityModeWrist", velocityMode);
        trapServo.setAngle(servoPos);
        SmartDashboard.putNumber("Servo Angle", trapServo.getAngle());

        if(velocityMode == true) {
            wristMotor.set(wristSpeed);
        }
        else {
            wristMotor.set(setPosition(setpoint));
        }
        trapServo.setAngle(servoPos);
    }

    /**
     * 
     */
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
    public double getServoPos(){
        return servoPos;
    }
    public void setServoPos(double servoPos){
        this.servoPos = servoPos;
    }



    
    public void setWristVelocity(double wristSpeed){
        this.wristSpeed = wristSpeed;
    }

    public void setWristVelocityWristFeed(double wristSpeed){
        this.wristSpeed = wristSpeed + wristPostionFeedForward.calculate(getPosition(), getVelocity());
    }

    public void setSetPoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void setVoltage(double voltage){
        wristMotor.setVoltage(voltage);
    }

    public void setVelocityMode(boolean isVelocityMode){
        velocityMode = isVelocityMode;
    }

    private double setPosition(double setpoint) {
		double output = (wristPostionFeedForward.calculate(getPosition(), getVelocity()))
        - wristPositionController.calculate(posToDegrees(), setpoint);
		// if (output >= 1){
		// 	output = 0.1;
		// } else if (output <= -1) {
		// 	output = -0.1;
		// }
		// double output = 0.1 * sin(getPositionDegrees()) +
		// armController.calculate(getPosition(), setpoint);
		// masterArmMotor.set(output);

		SmartDashboard.putNumber("wrist output", output);
        SmartDashboard.putNumber("WristMotorOutput", wristMotor.get());

        return output;
	}

    /**
     * Get the current setpoint of the wrist
     * @return the setpoint of the wrist as a double
     */
    public double getSetpoint(){
        return setpoint;
    }

    public double getVelocity(){
        return wristMotor.getVelocity().getValueAsDouble();
    }

    public double getPosition(){
        return wristMotor.getPosition().getValueAsDouble();
    }

    public TalonFX getWristMotor(){
        return wristMotor;
    }
    
    public double getVoltage() {
        // TODO Auto-generated method stub
        return getWristMotor().getMotorVoltage().getValueAsDouble();
    }

/**
 * Determines if we are within the wrist setpoint allowed error.
 * @return True if yes, false if no
 */
    public boolean nearSetpoint(){
         return (Math.abs((wristPostionFeedForward.calculate(getPosition(), getVelocity()) + 
         wristPositionController.calculate(getPosition(),setpoint))) / MAX_VOLTAGE) < WRIST_NEAR_SETPOINT_ERROR;
    }

    public boolean isClear(){
        return true;
    }

    public double posToDegrees(){
        return (getPosition() * WRIST_GEAR_RATIO)  + WRIST_DEGREES_TO_0;
    }

    /**
     * 
     * @return
     */
    public static Wrist getInstance(){
        if (wrist==null){
            wrist = new Wrist();
        }
        return wrist;
    }
}
