package ca.mcgill.ecse211.zipline;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ZiplineLab {
	protected static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	protected static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	protected static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(
			LocalEV3.get().getPort("C"));

	private static final Port usPort = LocalEV3.get().getPort("S1");

	public static TextLCD t = LocalEV3.get().getTextLCD();

	// Values based on 360 degree turn test
	protected static final double WHEEL_RADIUS = 2.093;
	protected static final double TRACK = 14.8;

	protected static final int ROTATIONSPEED = 100;
	protected static final int ACCELERATION = 1000;
	protected static final int FORWARDSPEED = 150;

	private static final int FILTER_OUT = 23;
	private static int filterControl;

	protected static final double BOT_LENGTH = 14.3;

	protected static final double TILE_LENGTH = 30.48;

	private static int x0 = 0;
	private static int y0 = 0;
	private static int xC = 0;
	private static int yC = 0;
	private static int corner = 0;

	public static void main(String[] args) {
		//set up screen for use
		int option = 0;
		final TextLCD screen = LocalEV3.get().getTextLCD();
		
		//set up us sensor
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from												// this instance
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are

		
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryDisplay odoDispl = new OdometryDisplay(odometer, screen);
		Navigation nav = new Navigation(odometer, leftMotor, rightMotor);
		UltrasonicLocalization usLocal = new UltrasonicLocalization(leftMotor, rightMotor, odometer);
		UltrasonicPoller usPoller = new UltrasonicPoller(usSensor, usData, usLocal);
		
		
		do {
			printMenu();
			option = Button.waitForAnyPress();
		} while (option != Button.ID_LEFT && option != Button.ID_RIGHT); // and wait for a button press. The button

		//pass points to light localization
		int[] points = { x0, y0, xC, yC, corner };
		LightLocalization lightLocal = new LightLocalization(leftMotor, rightMotor, sensorMotor, odometer, nav, points);
		LightPoller lp = new LightPoller(lightLocal);
		GameController gc = new GameController(lightLocal, usLocal, lp, usPoller);
		
		switch (option) {
		case Button.ID_LEFT:
			odometer.start();
//			usPoller.start();
//			odoDispl.start();
//			screen.clear();
//			usLocal.start();
//			lightLocal.start();
			t.clear();
//			lp.start();
			gc.start();
			break;
		case Button.ID_RIGHT:
//			UltrasonicPoller usPoll = new UltrasonicPoller(usSensor, usData, usLocal);
//			odometer.start();
//			usPoll.start();
//			odoDispl.start();
//			screen.clear();
//			usLocal.start();
//			Button.waitForAnyPress();
//			lightLocal.start();
			break;
		default:
			System.out.println("Error - invalid button"); // None of the above - abort
			System.exit(-1);
			break;
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

	private static void printMenu() {
		boolean enter = false;
		t.clear();
		int counter = 0;

		x0 = printXY("X0: ", 0, 0);
		y0 = printXY("Y0: ", 0, 1);
		xC = printXY("XC: ", 0, 2);
		yC = printXY("YC: ", 0, 3);

		enter = false;
		counter = 0;
		t.drawString("Corner: ", 0, 4);
		while (!enter) {
			int buttonPressed = Button.waitForAnyPress();
			if (buttonPressed != Button.ID_ENTER) {
				if (buttonPressed == Button.ID_DOWN && counter == 0) {
					counter = 0;
				} else if (buttonPressed == Button.ID_DOWN && counter > 0) {
					counter--;
				} else if (buttonPressed == Button.ID_UP && counter == 3) {
					counter = 3;
				} else if (buttonPressed == Button.ID_UP && counter < 3) {
					counter++;
				}
				t.drawInt(counter, 8, 4);
			} else {
				corner = counter;
				enter = true;
			}
		}

		t.clear(); // the screen at initialization
		t.drawString("left = fallingEdge", 0, 0);
		t.drawString("right = risingEdge", 0, 1);
	}

	public static int printXY(String disp, int x, int y) {
		boolean enter = false;
		int counter = 0;
		t.drawString(disp, x, y);
		while (!enter) {
			int buttonPressed = Button.waitForAnyPress();
			if (buttonPressed != Button.ID_ENTER) {
				if (buttonPressed == Button.ID_DOWN && counter == 0) {
					counter = 0;
				} else if (buttonPressed == Button.ID_DOWN && counter > 0) {
					counter--;
				} else if (buttonPressed == Button.ID_UP && counter == 8) {
					counter = 8;
				} else if (buttonPressed == Button.ID_UP && counter < 8) {
					counter++;
				}
				t.drawInt(counter, 3, y);
			} else {
				enter = true;
				break;
			}
		}
		return counter;
	}

	protected static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	protected static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}

}
