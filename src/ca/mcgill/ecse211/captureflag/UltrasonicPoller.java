package ca.mcgill.ecse211.captureflag;

import lejos.robotics.SampleProvider;

public class UltrasonicPoller extends Thread {
	private SampleProvider us;
	private UltrasonicLocalization ul;
	private float[] usData;
	private volatile boolean kill = false;

	// simple poller for getting samples for ultrasonic sensor
	public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicLocalization ul) {
		this.us = us;
		this.ul = ul;
		this.usData = usData;
	}

	public void run() {
		int distance;
		while (!kill) {
			us.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
			ul.processUSData(distance); // now take action depending on value
			try {
				Thread.sleep(25);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}
	
	protected void killTask() {
		kill = true;
	}

}
