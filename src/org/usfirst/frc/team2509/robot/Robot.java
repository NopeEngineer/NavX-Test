/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2509.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot implements PIDOutput{
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	
	
	static final double kToleranceDegrees = 5.0f;
	
	WPI_TalonSRX DriveTrain_left1 = new WPI_TalonSRX(0);
	
	WPI_TalonSRX DriveTrain_left2 = new WPI_TalonSRX(2);
	
	WPI_TalonSRX DriveTrain_left3 = new WPI_TalonSRX(1);
	
	WPI_TalonSRX DriveTrain_right1 = new WPI_TalonSRX(3);
	
	WPI_TalonSRX DriveTrain_right2 = new WPI_TalonSRX(4);
	
	WPI_TalonSRX DriveTrain_right3 = new WPI_TalonSRX(5);
	
	SpeedControllerGroup DriveTrain_Left = new SpeedControllerGroup(
			DriveTrain_left1,DriveTrain_left2,DriveTrain_left3);
	
	SpeedControllerGroup DriveTrain_Right = new SpeedControllerGroup(
			DriveTrain_right1,DriveTrain_right2,DriveTrain_right3);
	
	AHRS ahrs;
	DifferentialDrive drive;
	Joystick stick;
	PIDController turnController;
	double rotateToAngleRate;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		
		drive = new DifferentialDrive(DriveTrain_Left, DriveTrain_Right);
		stick = new Joystick(0);
		
		ahrs = new AHRS(SPI.Port.kMXP);
		
		turnController = new PIDController(kP, kI, kD, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-0.5, 0.5);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		
		SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
		
	  
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
	}

	@Override
	public void autonomousInit() {
		drive.setSafetyEnabled(true);
		while(ahrs.getAngle()<2.5&&ahrs.getAngle()>-2.5) {
			SmartDashboard.putNumber("Gyro", ahrs.getAngle());
	        boolean rotateToAngle = false;
	        turnController.setSetpoint(0.0f);
	        rotateToAngle = true;
	        double currentRotationRate;
	        turnController.enable();
	        currentRotationRate = rotateToAngleRate;
	        drive.arcadeDrive(0, currentRotationRate);
		}
		drive.arcadeDrive(0, 0);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putData("PID Controller",turnController);
		drive.setSafetyEnabled(true);
	      while (isOperatorControl() && isEnabled()) {
	    	  SmartDashboard.putNumber("Gyro", ahrs.getAngle());
	          boolean rotateToAngle = false;
	         
	          if ( stick.getRawButton(1)) {  //this is just to zero the gyro so you don't have to restart code every time
	              ahrs.reset();
	          }
	          if ( stick.getRawButton(2)) {
	              turnController.setSetpoint(0.0f);
	              rotateToAngle = true;
	          } else if ( stick.getRawButton(3)) {
	              turnController.setSetpoint(90.0f);
	              rotateToAngle = true;
	          } else if ( stick.getRawButton(4)) {
	              turnController.setSetpoint(179.9f);
	              rotateToAngle = true;
	          } else if ( stick.getRawButton(5)) {
	              turnController.setSetpoint(-90.0f);
	              rotateToAngle = true;
	          }
	          double currentRotationRate;
	          
	          if ( rotateToAngle ) {
	              turnController.enable();
	              currentRotationRate = rotateToAngleRate;
	          } else {
	              turnController.disable();
	              currentRotationRate = stick.getTwist();
	          }
	          drive.arcadeDrive(0, currentRotationRate);
	          
	      }
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
	}

	@Override
	public void pidWrite(double output) {
		 rotateToAngleRate = output;
		// TODO Auto-generated method stub
		
	}
}
