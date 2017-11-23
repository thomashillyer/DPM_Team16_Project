package ca.mcgill.ecse211.captureflag;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;
import java.util.List;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class FlagDetection {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	private Navigation nav;
	private int flagColor;
	private double x;
	private double y;
	private int currentDistance;
	private List<Point2D.Double> points;
	private int previousDistance = 255;
	private double previousTheta = 0;

	private Object lock = new Object();
	private volatile boolean isFlagDetecting = true;

	public FlagDetection(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			Navigation nav, double x, double y, int flagColor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		// this.x = x;
		// this.y = y;
		points = new ArrayList<>();
		this.nav = nav;
		this.flagColor = flagColor;
	}

	boolean firstFlagSearching = true;
	boolean lastFlagSearching = true;

	boolean foundRising = false;
	boolean foundFalling = false;

	protected void processUSData(int distance) {
		// protected void processUSData(float[] ultrasonicData) {
		// float avgDist = 0;
		if (this.isFlagDetecting()) {
			// filter ultrasonic sensor data
			// currentDistance = (int) (avgDist/ultrasonicData.length);
			// previou for (int i = 0; i < ultrasonicData.length; i++) {
			// currentDistance = CaptureFlag.filter((int) ultrasonicData[i] * 100);
			// avgDist += currentDistance;
			// }
			// sTheta;
			currentDistance = CaptureFlag.filter(distance);
			// Sound.beep();
			// Sound.beep();
			// System.out.println("usle " +ultrasonicData.length);
			//System.out.println("dist " + currentDistance);

			double RANGE_SAFETY_SCALE = 1.5;
			// calculated by using the diagonal of a block placed at the far corner of a
			// square, any angle less than this is not a new flag
			double MIN_ANGLE_SEPARATION = 0.3835967;
			double theta = 0, f_dX = 0, f_dY = 0, r_dX = 0, r_dY = 0, dX = 0, dY = 0;
			double first_dX = 0, first_dY = 0, last_dX = 0, last_dY = 0;
			// System.out.println("distance " +distance);
			// within the two blocks.
			double flagRangeLimit = RANGE_SAFETY_SCALE * Math.sqrt(2) * CaptureFlag.TILE_LENGTH;

			if (currentDistance < flagRangeLimit) {

				// Sound.buzz();

				// falling edge
				if (!foundFalling && firstFlagSearching && (previousDistance - currentDistance) > 10) {
					// firstFlagSearching = false;
					foundFalling = true;
					// first flag
					theta = odometer.getTheta();

					f_dX = currentDistance * Math.sin(theta);
					f_dY = currentDistance * Math.cos(theta);
					// first_dX = currentDistance * Math.cos(theta);
					// first_dY = currentDistance * Math.sin(theta);
					// points.add(new Point2D.Double(x + first_dX, y + first_dY));
					// Sound.beep();

				}
				// rising edge
				else if (!foundFalling && lastFlagSearching && (currentDistance - previousDistance) > 10) {
					// lastFlagSearching = false;
					foundFalling = true;
					// last flag
					theta = odometer.getTheta();

					r_dX = currentDistance * Math.sin(theta);
					r_dY = currentDistance * Math.cos(theta);
					// last_dX = currentDistance * Math.cos(theta);
					// last_dY = currentDistance * Math.sin(theta);
					// points.add(new Point2D.Double(x + last_dX, y + last_dY));
					// Sound.beep();
				}
				// store the point as the approximate middle of the block (flag)
				dX = (f_dX + r_dX) / 2.0;
				dY = (f_dY + r_dY) / 2.0;
				if (foundFalling && foundRising) {
					points.add(new Point2D.Double(x + dX, y + dY));
					Sound.beep();
				}
				foundRising = false;
				foundFalling = false;
				f_dX = 0;
				f_dY = 0;
				r_dX = 0;
				r_dY = 0;
			}

			previousDistance = currentDistance;
			previousTheta = theta;
		}

	}

	public void processLightSensorData(float r, float g, float b) {
		// TODO Auto-generated method stub
		r = Math.round(r);
		g = Math.round(g);
		b = Math.round(b);

	}

	protected void findFlag(double x, double y) {
		// robot is assumed to be starting at an intersection in between the two
		// possible squares
		// we are provided with the top right and lower left points so we will have to
		// calculate that point.

		// |-------------|-------------|
		// |.............|.............|
		// |.............|.............|
		// |.............|.............|
		// |.............|.............|
		// |.............|.............|
		// |-------------X-------------|

		synchronized (lock) {
			isFlagDetecting = true;
		}

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);

		// TODO turn to the left to face the edge first
		// nav.turnTo()
		// rotate through 180 degrees to sweep the two squares for 'flags'
		leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 180), true);
		rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 180), false);

		synchronized (lock) {
			isFlagDetecting = false;
		}
		/*
		 * List<Integer> toRemove = new ArrayList<>();
		 * 
		 * // remove points that are too close together i.e. false positives // is weird
		 * because it prevents missin points for (int i = 0; i < points.size(); i++) {
		 * for (int j = i + 1; j < points.size(); j++) { if
		 * (Math.abs(points.get(i).getX() - points.get(j).getX()) < 2) {
		 * toRemove.add(j); } } }
		 * 
		 * Object[] pointArray = points.toArray();
		 * 
		 * for (int i : toRemove) { pointArray[i] = null; }
		 * 
		 * points = new ArrayList<>();
		 * 
		 * for (int i = 0; i < pointArray.length; i++) { if (pointArray[i] != null) {
		 * points.add((Point2D.Double) pointArray[i]); } }
		 * 
		 * // hopefully only 3 points System.out.println("points size " +
		 * points.size());
		 */

		// TODO fix travelling to flags and implement logic to check colour of flag
		for (Point2D.Double p : points) {
			travelToFlag(p);
			travelBackFromFlag();

		}

	}

	private void travelBackFromFlag() {
		// should reverse back to the original point
		leftMotor.rotate(-CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, CaptureFlag.BOT_LENGTH), true);
		rightMotor.rotate(-CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, CaptureFlag.BOT_LENGTH), false);

	}

	/**
	 * 
	 * @param p
	 */
	private void travelToFlag(Point2D.Double p) {

		nav.travelTo(p.getX() - 10, p.getY() - 10);
	}

	/**
	 * Returns whether the robot is currently detecting the flag or not
	 * 
	 * @return Boolean value indicating if the robot is in flag detecting mode or
	 *         not
	 */
	protected boolean isFlagDetecting() {
		// boolean result;
		synchronized (lock) {
			// return result = this.isFlagDetecting;
			return this.isFlagDetecting;
		}
		// return result;
	}

}
