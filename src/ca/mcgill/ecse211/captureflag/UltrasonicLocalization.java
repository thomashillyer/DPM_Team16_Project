package ca.mcgill.ecse211.captureflag;

import ca.mcgill.ecse211.captureflag.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The UltrasonicLocalization class is used to orient the robot to 0 degrees. It
 * is used only in the beginning of the competition, when the robot is still in
 * the corner.
 */

public class UltrasonicLocalization {

	private final int HORIZONTAL_CONST = 35;
	private final int HORIZONTAL_MARGIN = 2;

	private static double alphaAngle = 0.0;
	private static double betaAngle = 0.0;
	private static double deltaTheta = 0.0;

	private int distanceUS = 255;
	private int previousDistance;
	private int filterControl;
	private static final int FILTER_OUT = 10;
	private int counter = 0;
	
	private int risingCounter = 0;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private Object lock = new Object();
	
	private double fallingTheta;
	private double risingTheta;
	private boolean detectRaisingEdge = false;
	
	private volatile boolean isLocalizing = false;

	private boolean fallingEdgeDetected = false;
	
	/**
	 * The constructor for the UltrasonicLocalization class
	 * 
	 * @param leftMotor
	 *            An instance of the EV3LargeRegulatedMotor that controls the left
	 *            motor.
	 * @param rightMotor
	 *            An instance of the EV3LargeRegulatedMotor that controls the right
	 *            motor.
	 * @param odometer
	 *            An instance of the Odometer class.
	 */

	public UltrasonicLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}

	/**
	 * The falling and rising edges are detected in this method. The rising edge is
	 * when the sensor goes from facing the wall to away from the wall and the
	 * falling edge is from away from the wall to towards the wall. If no falling
	 * edge is detected and the distance drops below the constant then a falling
	 * edge is said to have been detected. Then if a falling edge is detected and
	 * the distance rises above our horizontal constant, a rising edge is said to be
	 * detected. The angle return by the odometer is then saved. Then, the method
	 * uses both angles to determine by how much the robot should turn to make it
	 * face north (0 degrees)
	 */
	protected void localize() {
	    CaptureFlag.t.clear();
		synchronized (lock) {
			isLocalizing = true;
		}

		leftMotor.stop();
		leftMotor.setAcceleration(CaptureFlag.ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(CaptureFlag.ACCELERATION);

		leftMotor.setSpeed(CaptureFlag.ROTATIONSPEED + 70);
		rightMotor.setSpeed(CaptureFlag.ROTATIONSPEED + 70);
		
		if(readUSDistance() >= HORIZONTAL_CONST + HORIZONTAL_MARGIN) {
		  fallingEdge();
		}
		else {
		  risingEdge();
		}
		
	}

	public int readUSDistance() {
		return this.distanceUS;
	}

	/**
	 * Filter any extraneous distances returned by the sensor
	 * 
	 * @param distance
	 *            the distance recorded by the US sensor
	 */
	public void processUSData(int distance) {
		this.previousDistance = this.distanceUS;
		if (betaAngle != 0.0) {
			distance = 255;
		} else {
			if (distance >= 255) {
				// We have repeated large values, so there must actually be
				// nothing
				// there: leave the distance alone
				this.distanceUS = 255;
			} else {
				// distance went below 255: reset filter and leave
				// distance alone.
				filterControl = 0;

				this.distanceUS = distance;

			}
		}
	}
/**
 * 
 * @return a boolean indicating if the robot is in ultrasonic localization
 */
	protected boolean isLocalizing() {
		boolean result;
		synchronized (lock) {
			result = this.isLocalizing;
		}
		return result;
	}
	
	/**
	 * Calculates a correction angle based on a rising edge formula. This method of localization is preferred if the robot is facing a wall.
	 */
	private void risingEdge() {
		rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), true);
		leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), true);

		// detect falling and rising edge, calculate the angle by which the
		// robot should turn to face north
		// As long as the robot is moving, don't leave this loop
				while (leftMotor.isMoving() && rightMotor.isMoving()) {

					if ((this.distanceUS >= HORIZONTAL_CONST - HORIZONTAL_MARGIN) && detectRaisingEdge == false && risingCounter == 0) {
						detectRaisingEdge = true;
						risingTheta = odometer.getTheta();
						risingCounter++;
						Sound.beep();
					} else if (this.distanceUS < HORIZONTAL_CONST + HORIZONTAL_MARGIN && detectRaisingEdge == true && risingCounter == 1) {
						detectRaisingEdge = false;
						fallingTheta = odometer.getTheta();
						risingCounter++;
						Sound.beep();
					}
				}

				if (fallingTheta < risingTheta) {
					deltaTheta = (5.0 * Math.PI / 4.0) - (fallingTheta + risingTheta) / 2.0;
				} else {
					deltaTheta = (Math.PI / 4.0) - (fallingTheta + risingTheta) / 2.0;
				}

				odometer.setTheta(odometer.getTheta() + deltaTheta);

		// turn the robot by the angle by which it will make it head north
		rightMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, deltaTheta * 180.0 / Math.PI), true);
		leftMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, deltaTheta * 180.0 / Math.PI), false);
	}
	
	
/**
 * Calculates a correction angle based on a falling edge formula. This method of localization is preferred if the robot is facing away from a wall.
 */
	private void fallingEdge() {
		leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), true);
		rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 360), true);

		/*
		 * while robot is rotating, compare the distance returned by the ultrasonic
		 * sensor to the threshold, and if it is less than threshold, a falling edge is
		 * detected. Save the angle and set the flag to true, so when the distance gets
		 * bigger than the threshold, the program enters the second if statement and
		 * save the anlge of the rising edge.
		 */
		while ((rightMotor.isMoving() && leftMotor.isMoving())) {

			if (!fallingEdgeDetected && (this.distanceUS <= HORIZONTAL_CONST + HORIZONTAL_MARGIN)
					&& (this.distanceUS < this.previousDistance) && counter == 0) {
				alphaAngle = odometer.getTheta();
				Sound.beep();
				fallingEdgeDetected = true;
				counter++;

			} else if (fallingEdgeDetected && (this.distanceUS > HORIZONTAL_CONST - HORIZONTAL_MARGIN)
					&& (this.distanceUS > this.previousDistance) && counter == 1) {
				betaAngle = odometer.getTheta();
				Sound.beep();
				counter++;
				fallingEdgeDetected = false;
			}
		}

		// compute the odometer's angular offset
		if (alphaAngle <= betaAngle) {
			deltaTheta = 5.0 * Math.PI / 4.0 - (alphaAngle + betaAngle) / 2.0;
		} else if (alphaAngle > betaAngle) {
			deltaTheta = Math.PI / 4.0 - (alphaAngle + betaAngle) / 2.0;
		}
		leftMotor.rotate(
				-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, deltaTheta * 180 / Math.PI),
				true);
		rightMotor.rotate(
				CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, deltaTheta * 180 / Math.PI),
				false);
		odometer.setTheta(0);

		synchronized (lock) {
			isLocalizing = true;
		}
	}
}