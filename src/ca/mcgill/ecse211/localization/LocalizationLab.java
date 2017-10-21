package ca.mcgill.ecse211.localization;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LocalizationLab {
	protected static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	protected static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	protected static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(
			LocalEV3.get().getPort("C"));

	private static final Port usPort = LocalEV3.get().getPort("S1");

	public static TextLCD t = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RADIUS = 2.093;
	public static final double TRACK = 16;

	private static final int FILTER_OUT = 23;
	private static int filterControl;
	
	protected static final double robotLength = 14.3;
	
	protected static final double TILE_LENGTH = 30.48;

	public static void main(String[] args) {

		int option = 0;
		printMenu(); // Set up the display on the EV3 screen
		while (option == 0) // and wait for a button press. The button
			option = Button.waitForAnyPress(); // ID (option) determines what type of control to use

		int buttonChoice;
		Odometer odometer = new Odometer(leftMotor, rightMotor);

		final TextLCD t = LocalEV3.get().getTextLCD();
		OdometryDisplay disp = new OdometryDisplay(odometer, t);
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
																	// this instance
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are

		Navigation nav = new Navigation(odometer, leftMotor, rightMotor);
		
		LightLocalizer lu = new LightLocalizer(odometer, leftMotor, rightMotor, nav);

		LightPoller lPoller = new LightPoller(lu);

		UltrasonicLocalizer ul = new UltrasonicLocalizer(odometer, lu, lPoller, rightMotor, leftMotor);

		switch (option) {
		case Button.ID_LEFT:
			UltrasonicPoller usPoller = new UltrasonicPoller(usSensor, usData, ul);
			odometer.start();
			usPoller.start();
			disp.start();
			t.clear();
			ul.fallingEdge();
			break;
		case Button.ID_RIGHT:
			UltrasonicPoller usPoll = new UltrasonicPoller(usSensor, usData, ul);
			odometer.start();
			usPoll.start();
			disp.start();
			t.clear();
			ul.risingEdge();
			break;
		default:
			System.out.println("Error - invalid button"); // None of the above - abort
			System.exit(-1);
			break;
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

	private static void printMenu() {
		t.clear(); // the screen at initialization
		t.drawString("left = fallingEdge", 0, 0);
		t.drawString("right = risingEdge", 0, 1);
	}
	
	protected static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    protected static int convertAngle(double radius, double TRACK, double angle) {
        return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
    }

}
