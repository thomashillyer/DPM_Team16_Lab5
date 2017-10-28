package ca.mcgill.ecse211.zipline;

import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {
  private LightLocalization li;
  private volatile boolean kill = false;

  //simple poller for getting samples for light sensor
  public LightPoller(LightLocalization li) {
    this.li = li;
  }
  
  //polls light sensor
  public void run() {
	  while(!kill) {
		  li.processData();
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
