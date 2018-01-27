/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2509.robot;

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
	
	static final double kToleranceDegrees = 2.0f;
	
	Talon DriveTrain_left1 = new Talon(0);
	
	Talon DriveTrain_left2 = new Talon(2);
	
	Talon DriveTrain_left3 = new Talon(1);
	
	Talon DriveTrain_right1 = new Talon(3);
	
	Talon DriveTrain_right2 = new Talon(4);
	
	Talon DriveTrain_right3 = new Talon(5);
	
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
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		
		drive.setSafetyEnabled(false);
		turnController.setSetpoint(90f); //turns to 90 degrees for test purposes
		
		//OK, I'm  of bored right now so I'm gonna try and make this drive straight
		
		ahrs.reset();
		while  (isAutonomous());
			ahrs.getAngle();
			drive.arcadeDrive(-1.0, kP);
			Timer.delay(0.005);
			drive.arcadeDrive(0.0, 0.0);
	}

	
	

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		 
		drive.setSafetyEnabled(true);
	      while (isOperatorControl() && isEnabled()) {
	          boolean rotateToAngle = false;
	         
	          if ( stick.getRawButton(1)) {  //this is just to zero the gyro so you don't have to restart code every time
	              ahrs.reset();
	          }
	          double currentRotationRate;
	          if ( rotateToAngle ) {
	              turnController.enable();
	              currentRotationRate = rotateToAngleRate;
	          } else {
	              turnController.disable();
	              currentRotationRate = stick.getTwist();
	          }
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
