package ca.mcgill.ecse211.zipline;

import lejos.hardware.Button;

public class GameController extends Thread {
	
	private LightLocalization li;
	private UltrasonicLocalization us;
	private LightPoller lp;
	private UltrasonicPoller usPoller;

	public GameController(LightLocalization li, UltrasonicLocalization us, LightPoller lp, UltrasonicPoller usPoller) {
		this.li = li;
		this.us = us;
		this.lp = lp;
		this.usPoller = usPoller;
	}
	
	public void run() {
		usPoller.start();
		us.localize();
		usPoller.killTask();
		
		Button.waitForAnyPress();
		
		lp.start();
		li.cornerLocalization();
		lp.killTask();
	}
	
}
