package ca.mcgill.ecse211.localization;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	private final EV3LargeRegulatedMotor leftMotor;

	private final EV3LargeRegulatedMotor rightMotor;
	
	private Navigation nav;

	private static final Port cs = LocalEV3.get().getPort("S2");

	private SensorModes csSensor = new EV3ColorSensor(cs);
	private SampleProvider csValue = csSensor.getMode("Red");

	private boolean isNavigating = false;
	
	private float oldValue = 0; // holds value of previous distance

	private float[] csData = new float[csSensor.sampleSize()];
	
	private double[] radians = new double[4];
	
	//line counter
	private int counter = 0;
	
	private int lineCount = 0;

	public LightLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation nav) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.nav = nav;
		odometer = odo;
	}

	public void run() {
		rightMotor.setSpeed(150);
		leftMotor.setSpeed(150);
		rightMotor.forward();
		leftMotor.forward();
		
	}

	protected void processData() {
		long correctionStart, correctionEnd;
		correctionStart = System.currentTimeMillis();

		csValue.fetchSample(csData, 0);

		float value = csData[0] * 1000;

		// calculate derivative
		float diff = value - oldValue;

		// update old distance
		oldValue = value;

		// check if derivative is negative to see if a black line is detected
		if (diff < -70) {
			rightMotor.stop(true);
			leftMotor.stop(false);
			if(counter == 0) {
				odometer.setY(LocalizationLab.robotLength);
				counter++;
				
				rightMotor.rotate(-LocalizationLab.convertDistance(LocalizationLab.WHEEL_RADIUS, 1.5*LocalizationLab.robotLength), true);
				leftMotor.rotate(-LocalizationLab.convertDistance(LocalizationLab.WHEEL_RADIUS, 1.5*LocalizationLab.robotLength), false);
				
				rightMotor.rotate(-LocalizationLab.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 90), true);
				leftMotor.rotate(LocalizationLab.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 90), false);
				
				rightMotor.forward();
				leftMotor.forward();
				
			} else if(counter == 1) {
				odometer.setX(LocalizationLab.robotLength);
				counter++;
				
				nav.travelTo(0, 0);
				
				leftMotor.rotate(-UltrasonicLocalizer.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360), true);
				rightMotor.rotate(UltrasonicLocalizer.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360), false);
				
				if(lineCount < 4) {
					radians[lineCount] = odometer.getTheta();
				}
				
				//increment line counter
				lineCount++;
				
				// we have crossed all 4 lines and can do the calculation now
				if(lineCount == 4) {
					double thetaY = (radians[0] - radians[2])/2;
					double thetaX = (radians[1] - radians[3])/2;
					
					double correctionTheta = 90+(thetaY*180/Math.PI)-(radians[0]*180/Math.PI-180);
									
					leftMotor.stop(true);
					rightMotor.stop(false);
					
					 //correct theta					
					leftMotor.rotate(-UltrasonicLocalizer.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 25+correctionTheta), true);
					rightMotor.rotate(UltrasonicLocalizer.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 25+correctionTheta), false);
				}
				
				Button.waitForAnyPress();
				
				lineCount = 0;
				
				nav.travelTo(0, 1);
				
				leftMotor.rotate(-UltrasonicLocalizer.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360), true);
				rightMotor.rotate(UltrasonicLocalizer.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360), false);
				
				if(lineCount < 4) {
					radians[lineCount] = odometer.getTheta();
				}
				
				//increment line counter
				lineCount++;
				
				// we have crossed all 4 lines and can do the calculation now
				if(lineCount == 4) {
					double thetaY = (radians[0] - radians[2])/2;
					double thetaX = (radians[1] - radians[3])/2;
					
					double correctionTheta = 90+(thetaY*180/Math.PI)-(radians[0]*180/Math.PI-180);
									
					leftMotor.stop(true);
					rightMotor.stop(false);
					
					 //correct theta					
					leftMotor.rotate(-UltrasonicLocalizer.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 25+correctionTheta), true);
					rightMotor.rotate(UltrasonicLocalizer.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 25+correctionTheta), false);
				}
				
				nav.travelTo(1, 1);
			}
			
		}
	}
	
	
	//assumes on 45 deg line
	//calculates a distance it has to travel
	private void travelTo(double x, double y) {
		double dX = x - odometer.getX();
		double dY = y - odometer.getY();

		double minimumAngle = Math.atan2(dX, dY) - odometer.getTheta();

		double distance = Math.hypot(dX, dY);

		leftMotor.setSpeed(UltrasonicLocalizer.ROTATION_SPEED);
		rightMotor.setSpeed(UltrasonicLocalizer.ROTATION_SPEED);
		leftMotor.rotate(UltrasonicLocalizer.convertDistance(LocalizationLab.WHEEL_RADIUS, distance), true);
		rightMotor.rotate(UltrasonicLocalizer.convertDistance(LocalizationLab.WHEEL_RADIUS, distance), false);
	}

}
