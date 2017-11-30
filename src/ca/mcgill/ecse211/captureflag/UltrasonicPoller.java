package ca.mcgill.ecse211.captureflag;

import lejos.robotics.SampleProvider;

/**
 * This class will instantiate a poller thread and run it so that we can get data samples from the
 * ultrasonic sensor
 */
public class UltrasonicPoller extends Thread {
  private SampleProvider us;
  private UltrasonicLocalization ul;
  private FlagDetection flag;
  private float[] usData;
  private volatile boolean kill = false;

  // simple poller for getting samples for ultrasonic sensor
  public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicLocalization ul,
      FlagDetection flag) {
    this.us = us;
    this.ul = ul;
    this.usData = usData;
    this.flag = flag;
  }

  public void run() {
    int distance;
    while (true) {
      while (!kill) {
        us.fetchSample(usData, 0); // acquire data
        distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
        if (ul.isLocalizing()) {
          ul.processUSData(distance); // now take action depending on value
        }

        if (flag.isFlagDetecting()) {
          flag.processUSData(distance);
        }

        try {
          Thread.sleep(25);
        } catch (Exception e) {
        } // Poor man's timed sampling
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

  /**
   * Allows thread to be restarted without deadlock
   */
  protected void restartTask() {
    kill = false;
  }
}
