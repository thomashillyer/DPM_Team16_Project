package ca.mcgill.ecse211.captureflag;

import java.util.Arrays;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class provides the light localization that can be used at any point to relocalize to the nearest grid intersection
 */

public class LightLocalization {
	//object instances
	private Navigation nav;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	//data
	private int filterCounter = 0;
	private float oldValue = 0;
	private int derivativeThreshold = -30;
	private int lineCounter = 0;
	private double xminus, xplus, yminus, yplus;
	private double thetax, thetay;
	private double x, y;
	private double deltaThetaY;

	//TODO Remove x0, y0, xC, yC, and corner as the 
	private int x0;
	private int y0;
	private int xC;
	private int yC;
	private int corner;

	private double currX = 0.0;
	private double currY = 0.0;
	private double currTheta = 0.0;
	
	private boolean detectSingleLine = false;
	private boolean detectFourLines = false;
	private boolean cornerLocalization = false;
	
	private double[] lines = new double[4];

	/**
	 * The constructor for LightLocalization class.
	 * @param leftMotor An instance of the EV3LargeRegulatedMotor that controls the left motor.
	 * @param rightMotor An instance of the EV3LargeRegulatedMotor that controls the right motor.
	 * @param odometer An instance of the Odometer class.
	 * @param nav An instance of the Navigation class.
	 */
	public LightLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			Odometer odometer, Navigation nav) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.nav = nav;
	}

	
	/**
	 *This method gets input from the LightPoller instance that is running. 
	 *The method will check if the difference between the passed in value and the previously passed value is less than the threshold for detecting a black line.
	 *It then handles what happens if a black line is detected depending on the state of the robot.
	 *@param value A value that is passed in from the LightPoller class. The value is retrieved by sampling the color sensor.
	 */
	protected void processData(int value) {
		long correctionStart, correctionEnd;
		correctionStart = System.currentTimeMillis();
		
		// computing the derivative at each point
		float diff = value - oldValue;

		// storing the current value, to be able to get the derivative on
		// the next iteration
		oldValue = value;
		if (diff < derivativeThreshold && filterCounter == 0) {
			filterCounter++;
			Sound.beep();
			if(detectFourLines) {
				lineCounter++;
				
				if(lineCounter-1 < 4)
					lines[lineCounter-1] = odometer.getTheta();
				
				if (lineCounter == 1) {
//					System.out.println(1 + " odo: " + odometer.getTheta());
					xminus = odometer.getTheta();

				} else if (lineCounter == 2) {
//					System.out.println(2+ " odo: " + odometer.getTheta());
					yplus = odometer.getTheta();

				} else if (lineCounter == 3) {
//					System.out.println(3+ " odo: " + odometer.getTheta());
					xplus = odometer.getTheta();

				} else if (lineCounter == 4) {
//					System.out.println(4+ " odo: " + odometer.getTheta());
					yminus = odometer.getTheta();
				}
			}else if(detectSingleLine) {
				detectSingleLine = false;
			}
		} else if (diff < derivativeThreshold && filterCounter > 0) {
			filterCounter++;
		} else if (diff > derivativeThreshold) {
			filterCounter = 0;
		}
	}

	/**
	 * This method localizes specifically from the starting corner of the board. The following are the steps it takes:
	 * <ol> 
	 * <li>It turns the robot 45 degrees</li> 
	 * <li>The robot goes forward until a line is detected</li>
	 * <li>The robot reverses by the length of the robot</li>
	 * <li>The robot rotates 360 degrees to detect four black lines</li>
	 * <li>The distance and angle offset are calculated</li>
	 * <li>Corrects the robots orientation</li>
	 * </ol>
	 */
	protected void cornerLocalization() {
		detectSingleLine = true;
		cornerLocalization = true;
		// Set the acceleration of both motor
		leftMotor.stop();
		leftMotor.setAcceleration(CaptureFlag.ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(CaptureFlag.ACCELERATION);

		// Adjust the robot position, before it starts to rotate to ensure that
		// the light sensor will cross 4 black lines
		adjustRobotStartingPosition();

		// set the robot wheel's rotation speed to both motors
		leftMotor.setSpeed(CaptureFlag.ROTATIONSPEED);
		rightMotor.setSpeed(CaptureFlag.ROTATIONSPEED);

		detectFourLines = true;
		
		// rotate the robot 360 degrees
		leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), true);
		rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), false);
		
		detectFourLines = false;
		
		correctOdometer();
		nav.travelTo(0, 0);
		nav.turnTo(-(odometer.getTheta() + (4 * Math.PI / 180)));
		
		odometer.setX(0);
		odometer.setY(0);
		cornerLocalization = false;
		//TODO set odometer based on corner passed in from server
	}

	protected void anyPointLocalization() {
		lineCounter = 0;
		detectFourLines = true;
		leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), true);
		rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), false);
		detectFourLines = false;
		
		correctOdometer();
		Button.waitForAnyPress();
		nav.travelTo(0, 0);
		nav.turnTo(-(odometer.getTheta() + (4 * Math.PI / 180)));
	}

	/**
	 * This method calculates the robot's actual x and y position using the angle
	 * values determined earlier.Using trigonometry, the theta by
	 * which the odometer is off is calculated. Then, it updates the x, y, and theta values of the
	 * odometer in order to fix them. Then, it uses the navigation travelTo method
	 * to travel to the inputed destination.
	 */
	private void correctOdometer() {
//		System.out.println("X- : " + xminus);
//		System.out.println("Y- : " + yminus);
//		System.out.println("X+ : " + xplus);
//		System.out.println("Y+ : " + yplus);
		
		Arrays.sort(lines);
////		System.out.println("the sorted array:");
//		for(double d : lines) {
//			System.out.println(d);
//		}
		
		double theta = lines[0] * 180/Math.PI;
		
		if(theta >= 0 && theta < 20) {
			thetay = lines[0] - lines[2];
			thetax = lines[3] - lines[1];
		} else {
			thetay = lines[3] - lines[1];
			thetax = lines[2] - lines[0];
		}
		
		this.x = -CaptureFlag.BOT_LENGTH * Math.cos(thetay / 2.0);
		this.y = -CaptureFlag.BOT_LENGTH * Math.cos(thetax / 2.0);
		deltaThetaY = (Math.PI / 2.0) - yminus + Math.PI + (thetay / 2.0);
		
		if(cornerLocalization) {
			odometer.setX(this.x);
			odometer.setY(this.y);
		}
		odometer.setTheta(odometer.getTheta() + deltaThetaY);

	}

	/**
	 * This method adjusts the robot position before it starts rotating. It ensures
	 * that the light sensor, mounted on the back of the robot, runs over 4 black
	 * lines. It order to so, the robot, facing north after the ultrasonic
	 * localization, moves forward until it detects a line, and then move backwards
	 * 1.5 times its center distance. This, ensures that the robot is close enough
	 * to the horizontal black line facing the robot. Then, the method makes the
	 * robot turn 90 degrees and repeat the same procedure, so it can be placed
	 * close enough to the vertical line. Now the robot it close enough to the first
	 * intersection.
	 */
	private void adjustRobotStartingPosition() {
		// Set the wheel's rotation speed to ROTATESPEED
		leftMotor.setSpeed(CaptureFlag.ROTATIONSPEED);
		rightMotor.setSpeed(CaptureFlag.ROTATIONSPEED);

		// Rotate the robot by 45 degrees
		leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 45), true);
		rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 45), false);

		while(detectSingleLine) {
			leftMotor.forward();
			rightMotor.forward();
		}
		
		leftMotor.stop(true);
		rightMotor.stop(true);
		
		// Move the robot backwards 1.5 * its center distance
		rightMotor.rotate(-CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, 1.15 * CaptureFlag.BOT_LENGTH), true);
		leftMotor.rotate(-CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, 1.15 * CaptureFlag.BOT_LENGTH), false);
	}
}
