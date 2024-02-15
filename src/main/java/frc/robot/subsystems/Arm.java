package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.util.Constants.Arm.*;
import static frc.robot.util.Constants.RobotSpecs.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.RobotSpecs;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Arm extends SubsystemBase {
    
    private static Arm arm;

    //left is main motor
    private static TalonFX masterArmMotor;
    private static TalonFX followArmMotor;

    private double armSpeed;
    private double setpoint;

    private ProfiledPIDController armPositionController;
    // private SimpleMotorFeedforward armPostionFeedForward;
        private ArmFeedforward armPostionFeedForward;


    public Arm() {
        masterArmMotor = new TalonFX(MASTER_ARM_MOTOR);
        followArmMotor = new TalonFX(FOLLOW_ARM_MOTOR);

        followArmMotor.setControl(new Follower(MASTER_ARM_MOTOR, true));

        armPositionController = new ProfiledPIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D, 
        new Constraints(POSITION_PID_V, POSITION_PID_A));

        // armPostionFeedForward = new SimpleMotorFeedforward(POSITION_FF_S, POSITION_FF_V, POSITION_FF_A);
        armPostionFeedForward = new ArmFeedforward(POSITION_FF_S, POSITION_FF_G, POSITION_FF_V, POSITION_FF_A);

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
        masterArmMotor.set(armSpeed);
        SmartDashboard.putNumber("Arm Postion", getPosition());
        SmartDashboard.putNumber("Arm Position Degrees", posToDegrees());
        SmartDashboard.putNumber("Arm Setpoint", setpoint);

        setPosition(setpoint);
    }


    /**
     * 
     */
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    /**
     * 
     */
    public void setArmVelocity(double armSpeed){
        this.armSpeed = armSpeed;
    }

    public void setArmVelocityG(double armSpeed){
        this.armSpeed = armSpeed + POSITION_FF_G * Math.cos(Math.toRadians(posToDegrees()));
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

    // public boolean nearSetpoint(){
    //     return (Math.abs(armPostionFeedForward.calculate(getPosition(), getVelocity()) + 
    //      armPositionController.calculate(getPosition(),setpoint)) / MAX_VOLTAGE) < ARM_NEAR_SETPOINT_ERROR;
    // }
    public boolean nearSetpoint(){
        return (Math.abs(armPositionController.calculate(posToDegrees(), setpoint) + POSITION_FF_G * Math.cos(Math.toRadians(posToDegrees())) / MAX_VOLTAGE) < ARM_NEAR_SETPOINT_ERROR);
    }

    public boolean isClear(){
        return true;
    }

    public double posToDegrees(){
        return (getPosition() * ARM_GEAR_RATIO)  + ARM_DEGREES_TO_0;
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

    public double getVoltage() {
        // TODO Auto-generated method stub
        return getArmMaster().getMotorVoltage().getValueAsDouble();
    }

    private void setPosition(double setpoint) {
		double output = ( POSITION_FF_G * Math.cos(Math.toRadians(posToDegrees()) 
        + armPositionController.calculate(posToDegrees(), setpoint))
				/ RobotSpecs.MAX_VOLTAGE); // INVERTED
		if (output >= 1){
			output = 0.1;
		} else if (output <= -1) {
			output = -0.1;
		}
		// double output = 0.1 * sin(getPositionDegrees()) +
		// armController.calculate(getPosition(), setpoint);
		masterArmMotor.set(output);

		SmartDashboard.putNumber("arm output", output);
        SmartDashboard.putNumber("ActualMotorOutput", masterArmMotor.get());
	}
}
