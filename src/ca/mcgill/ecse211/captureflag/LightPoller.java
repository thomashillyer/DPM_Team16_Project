package ca.mcgill.ecse211.captureflag;

import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {
	private LightLocalization li;
	private SampleProvider cs;
	private float[] csData;
	private volatile boolean kill = false;

	// simple poller for getting samples for light sensor
	public LightPoller(SampleProvider cs, float[] csData, LightLocalization li) {
		this.li = li;
		this.cs = cs;
		this.csData = csData;
	}

	// polls light sensor
	public void run() {
		int value;
		while (!kill) {
			// fetching the values from the color sensor
			cs.fetchSample(csData, 0);

			// getting the value returned from the sensor, and multiply it by
			// 1000 to scale
			value = (int) (csData[0] * 1000);
			li.processData(value);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	protected void killTask() {
		kill = true;
	}

}
