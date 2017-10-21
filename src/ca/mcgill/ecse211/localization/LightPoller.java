package ca.mcgill.ecse211.localization;

import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {
  private LightLocalizer li;

  //simple poller for getting samples for light sensor
  public LightPoller(LightLocalizer li) {
    this.li = li;
  }
  
  //polls light sensor
  public void run() {
	  while(true) {
		  li.processData();
		  try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	  }
  }

}
