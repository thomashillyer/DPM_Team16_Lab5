package ca.mcgill.ecse211.localization;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class UltrasonicLocalizer  {
	private Odometer odo;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;
	private LightLocalizer lu;
	private LightPoller lPoller;

	//Motion constants
	protected static final int ROTATION_SPEED = 100;
	protected static final int ACCEL = 200;

	//Filter vars
	private static final int FILTER_OUT = 20;
	private static int filterControl;

	private static final double ERROR = 0.165;
	
	//Noise Margin and Distance 
	private static final int EDGE_DIST = 30;
	private static final int EDGE_MARGIN = 1;
	
	//state boolean
	private boolean rotating = false;

	//stores Theta, Distance
	private Map<Double, Integer> points = new LinkedHashMap<>();

	public UltrasonicLocalizer(Odometer odo, LightLocalizer lu, LightPoller lPoller, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor) {
		this.odo = odo;
		this.lPoller = lPoller;
		this.lu = lu;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
	}

	//basic filtering and populate the map
	protected void processUSData(int distance) {
		if (rotating) {
			if (distance >= 255 && filterControl < FILTER_OUT) {
				filterControl++;
				points.put(odo.getTheta(), 255);
			} else if (distance < 255) {
				filterControl = 0;
				points.put(odo.getTheta(), distance);
			}
		}
	}

	//rotates once clockwise and then counter clockwise
	//detect and store first falling edge for both rotations
	protected void fallingEdge() {
		double point1 = 0;

		double point2 = 0;

		rotateClock();

		stopMotors();
		
		//first falling edge clockwise
		point1 = iterateFallingMap();

		points = new LinkedHashMap<>();

		rotateCounter();

		stopMotors();

		//first falling edge counter clock
		point2 = iterateFallingMap();

		double avg = (point1 + point2)/2;

		double dTheta = 0;

		if (point1 > point2) {
			dTheta = (5 * Math.PI / 4 - avg) - ERROR;
		} else {
			dTheta = (Math.PI / 4 - avg) - ERROR;
		}

		//convert to degrees
		double turnAngle = dTheta * 180 / Math.PI;

		leftMotor.setAcceleration(ACCEL);
		rightMotor.setAcceleration(ACCEL);

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		//rotate to zero based on dTheta
		leftMotor.rotate(-(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, turnAngle) + convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 178)), true);
		rightMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, turnAngle) + convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 178), false);
		
		odo.setTheta(0);

		//start light sensor correction
		Button.waitForAnyPress();
		lPoller.start();
		lu.start();

	}

	//rotates once clockwise and then counter clockwise
	//detect and store first falling edge for both rotations
	protected void risingEdge() {
		double point1 = 0;

		double point2 = 0;

		rotateClock();

		stopMotors();

		//first rising edge clockwise
		point1 = iterateRisingMap();

		points = new LinkedHashMap<>();

		rotateCounter();

		stopMotors();

		//first rising edge counter clock wise
		point2 = iterateRisingMap();

		double avg = (point1 + point2)/2;

		double dTheta = 0;
		
		if (point1 > point2) {
			dTheta = (5 * Math.PI /4 - avg) - ERROR + .1;
		} else {
			dTheta = (Math.PI/4 - avg) - ERROR + .1;
		}

		//convert to degrees
		double turnAngle = dTheta * 180 / Math.PI;

		leftMotor.setAcceleration(ACCEL);
		rightMotor.setAcceleration(ACCEL);

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		//rotate to zero
		leftMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, turnAngle), true);
		rightMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, turnAngle), false);
		
		odo.setTheta(0);

		//start light sensor correction
		Button.waitForAnyPress();
		lPoller.start();
		lu.start();
	}

	//rotates robot clockwise 360 deg
	private void rotateClock() {
		rotating = true;
		leftMotor.setAcceleration(ACCEL);
		rightMotor.setAcceleration(ACCEL);

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360.0), true);
		rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360.0), false);

		rotating = false;
	}

	private void stopMotors() {
		leftMotor.stop(true);		rightMotor.stop(true);
	}

	//rotates robot counter clockwise 360 deg
	private void rotateCounter() {
		rotating = true;
		leftMotor.setAcceleration(ACCEL);
		rightMotor.setAcceleration(ACCEL);

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		leftMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360.0), true);
		rightMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360.0), false);
		rotating = false;
	}

	//method to find first falling edge within noise margin by looping through map
	private double iterateFallingMap() {
		int previousPoint = 0;
		int counter = 0;
		int currentDist = 0;
		double point1 = 0;
		double point2 = 0;
		double angle = 0;
		double returnPoint = 0;

		for (Map.Entry<Double, Integer> entry : points.entrySet()) {
			currentDist = entry.getValue();
			angle = entry.getKey();
			//find when it enters noise margin and slope is negative
			if ((currentDist <= EDGE_DIST + EDGE_MARGIN) && (currentDist - previousPoint < 0) && (counter == 0)) {
				// in noise margin
				point1 = angle;
				counter++;
			}
			//find when it exits noise margin and derivative is negative
			else if ((currentDist <= EDGE_DIST - EDGE_MARGIN) && (currentDist - previousPoint < 0)
					&& (counter == 1)) {
				point2 = angle;
				counter++;
				break;
			}
			previousPoint = currentDist;
		}

		returnPoint = (point1 + point2) / 2;

		return returnPoint;
	}

	//get first rising edge by looping through map
	private double iterateRisingMap() {
		int previousPoint = 0;
		int counter = 0;
		int currentDist = 0;
		double point1 = 0;
		double point2 = 0;
		double angle = 0;
		double returnPoint = 0;

		for (Map.Entry<Double, Integer> entry : points.entrySet()) {
			currentDist = entry.getValue();
			angle = entry.getKey();
			// in noise margin
			if ((currentDist >= EDGE_DIST - EDGE_MARGIN) && (currentDist - previousPoint > 0) && (counter == 0)) {
				point1 = angle;
				counter++;
			} else if ((currentDist >= EDGE_DIST + EDGE_MARGIN) && (currentDist - previousPoint > 0)
					&& (counter == 1)) {
				point2 = angle;
				counter++;
				break;
			}
			previousPoint = currentDist;
		}

		returnPoint = (point1 + point2) / 2;

		return returnPoint;
	}

	protected static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	protected static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}