package ca.mcgill.ecse211.captureflag;

import lejos.robotics.SampleProvider;

/**
 * This class will instantiate a poller thread and run it so that we can get
 * data samples from the light sensor
 */
public class LightPoller extends Thread {
	private LightLocalization li;
	private SampleProvider cs;
	private float[] csData;
	private FlagDetection flag;
	private volatile boolean kill = false;

	// simple poller for getting samples for light sensor
	public LightPoller(SampleProvider cs, float[] csData, LightLocalization li) {
		this.li = li;
		this.cs = cs;
		this.csData = csData;
		this.flag = null;
	}

	public LightPoller(SampleProvider cs, float[] csData, FlagDetection flag) {
		this.li = null;
		this.cs = cs;
		this.csData = csData;
		this.flag = flag;
	}

	// polls light sensor
	public void run() {
		int value;
		while (true) {
			while (!kill) {
				// fetching the values from the color sensor
				cs.fetchSample(csData, 0);

				// getting the value returned from the sensor, and multiply it by
				// 1000 to scale
				value = (int) (csData[0] * 1000);
				if (li != null) {
					li.processData(value);
				} else {
					//TODO implement actual flag detection code
					//flag.processLightSensorData(value); 
				}
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

	/**
	 * Allows thread to be stopped without deadlock
	 * 
	 */
	protected void killTask() {
		kill = true;
	}

	protected void restartTask() {
		kill = false;
	}

}
