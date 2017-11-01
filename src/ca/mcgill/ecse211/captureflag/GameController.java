package ca.mcgill.ecse211.captureflag;

import lejos.hardware.Button;

/**
 * This class will implement the overall game logic that will follow according to our software flowchart
 */
public class GameController extends Thread {
	
	private LightLocalization li;
	private UltrasonicLocalization us;
	private LightPoller lp;
	private UltrasonicPoller usPoller;
	private Navigation nav;

	//constructor
	public GameController(LightLocalization li, UltrasonicLocalization us, LightPoller lp, UltrasonicPoller usPoller, Navigation nav) {
		this.li = li;
		this.us = us;
		this.lp = lp;
		this.usPoller = usPoller;
		this.nav = nav;
	}
	
	public void run() {
		usPoller.start();
		us.localize();
		usPoller.killTask();
		
		Button.waitForAnyPress();
		
		lp.start();
		li.cornerLocalization();
		lp.killTask();
		
		//nav.travelTo(1, 1);
	}
	
}
