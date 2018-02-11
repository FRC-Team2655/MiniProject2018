package org.usfirst.frc.team2655.robot.subsystems;

import java.awt.Robot;

import org.usfirst.frc.team2655.robot.values.Values;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TestSubsystem extends Subsystem {

	private final PIDOutput m_output = new PIDOutput() {
    	@Override
    	public void pidWrite(double output) {
			SmartDashboard.putNumber(Values.ROTATE_PID, 0/*Robot.imu.getAngleZ()*/);
    		Math.abs(-1);
    		rotatePIDController.getError();
    		if(rotatePIDController.isEnabled()) {
    			rotatePIDController.disable();
    		}
    	}
    };

	private final PIDSource m_source = new PIDSource() {
    	@Override
    	public double pidGet() {
    		return 0;// Robot.imu.getAngleZ();
    	}

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
    };
	public final PIDController rotatePIDController;
	private double rotateSetpoint = 0;
	
    // Initialize your subsystem here
    public TestSubsystem() {
        super("Test Subsystem");
        rotatePIDController = new PIDController(0, 0, 0, 0, m_source, m_output);
		//rotatePIDController.setContinuous(false);
		this.addChild("RotateController", rotatePIDController);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}
