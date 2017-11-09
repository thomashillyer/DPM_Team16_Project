package ca.mcgill.ecse211.captureflag;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

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

	private Object lock = new Object();
	private volatile boolean isFlagDetecting = false;

	public FlagDetection(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			Navigation nav, double x, double y, int flagColor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.x = x;
		this.y = y;
		points = new ArrayList<>();
		this.nav = nav;
		this.flagColor = flagColor;
	}

	protected void processUSData(int distance) {
		if (isFlagDetecting()) {
			currentDistance = CaptureFlag.filter(distance);

			if (currentDistance - previousDistance < 0
					&& currentDistance < 2 * Math.sqrt(2) * CaptureFlag.TILE_LENGTH) {
				double theta = odometer.getTheta();
				double dX = currentDistance * Math.sin(theta);
				double dY = currentDistance * Math.cos(theta);
				points.add(new Point2D.Double(x + dX, y + dY));
			}

			previousDistance = currentDistance;
		}
	}

	protected void findFlag() {
		synchronized (lock) {
			isFlagDetecting = true;
		}

		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 90), true);
		rightMotor.rotate(-CaptureFlag.convertAngle(CaptureFlag.WHEEL_RADIUS, CaptureFlag.TRACK, 90), false);

		synchronized (lock) {
			isFlagDetecting = false;
		}
		System.out.println("Points: " + points.size());
		List<Integer> toRemove = new ArrayList<>();
		
		for (int i = 0; i < points.size(); i++) {
			for (int j = i + 1; j < points.size(); j++) {
				if (Math.abs(points.get(i).getX() - points.get(j).getX()) < 2) {
					toRemove.add(j);
				}
			}
		}

		Object[] pointArray = points.toArray();
		
		for(int i : toRemove) {
			pointArray[i] = null;
		}
		
		points = new ArrayList<>();
		
		for(int i = 0; i < pointArray.length; i++) {
			if(pointArray[i] != null) {
				points.add((Point2D.Double) pointArray[i]);
			}
		}
		
		System.out.println(points.size());
		for (Point2D.Double p : points) {
			System.out.println(OdometryDisplay.formattedDoubleToString(p.getX(), 3) + " , "
					+ OdometryDisplay.formattedDoubleToString(p.getY(), 3));
			nav.travelTo(p.getX() - 10, p.getY() - 10);
			leftMotor.rotate(-CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, CaptureFlag.BOT_LENGTH), true);
			rightMotor.rotate(-CaptureFlag.convertDistance(CaptureFlag.WHEEL_RADIUS, CaptureFlag.BOT_LENGTH), false);
		}

	}

	protected boolean isFlagDetecting() {
		boolean result;
		synchronized (lock) {
			result = this.isFlagDetecting;
		}
		return result;
	}
}
