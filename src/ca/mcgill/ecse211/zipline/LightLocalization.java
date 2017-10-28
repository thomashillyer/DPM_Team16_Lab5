package ca.mcgill.ecse211.zipline;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalization {

	private static final Port lightSampler = LocalEV3.get().getPort("S2");

	private SensorModes colosSamplerSensor = new EV3ColorSensor(lightSampler);
	private SampleProvider colorSensorValue = colosSamplerSensor.getMode("Red");

	private Navigation nav;

	private float[] colorSensorData = new float[colosSamplerSensor.sampleSize()];

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private EV3MediumRegulatedMotor sensorMotor;
	// data
	private int filterCounter = 0;
	private float oldValue = 0;
	private int derivativeThreshold = -30;
	private int lineCounter = 0;
	private double xminus, xplus, yminus, yplus;
	private double thetax, thetay;
	private double x, y;
	private double deltaThetaY;

	private int x0;
	private int y0;
	private int xC;
	private int yC;
	private int corner;

	private double currX = 0.0;
	private double currY = 0.0;
	private double currTheta = 0.0;
	
	private boolean detectSingleLine = false;
	private boolean detectFourLines = false;

	public LightLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3MediumRegulatedMotor sensorMotor, Odometer odometer, Navigation nav, int[] points) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.nav = nav;
		x0 = points[0];
		y0 = points[1];
		xC = points[2];
		yC = points[3];
		corner = points[4];
	}

	protected void processData() {
		long correctionStart, correctionEnd;
		correctionStart = System.currentTimeMillis();

		// fetching the values from the color sensor
		colorSensorValue.fetchSample(colorSensorData, 0);

		// getting the value returned from the sensor, and multiply it by
		// 1000 to scale
		float value = colorSensorData[0] * 1000;

		// computing the derivative at each point
		float diff = value - oldValue;

		// storing the current value, to be able to get the derivative on
		// the next iteration
		oldValue = value;
		if (diff < derivativeThreshold && filterCounter == 0) {
			filterCounter++;
			Sound.beep();
			if(detectFourLines) {
				lineCounter++;

				if (lineCounter == 1) {
					System.out.println(1 + " odo: " + odometer.getTheta());
					xminus = odometer.getTheta();

				} else if (lineCounter == 2) {
					System.out.println(2+ " odo: " + odometer.getTheta());
					yplus = odometer.getTheta();

				} else if (lineCounter == 3) {
					System.out.println(3+ " odo: " + odometer.getTheta());
					xplus = odometer.getTheta();

				} else if (lineCounter == 4) {
					System.out.println(4+ " odo: " + odometer.getTheta());
					yminus = odometer.getTheta();
				}
			}else if(detectSingleLine) {
				detectSingleLine = false;
			}
		} else if (diff < derivativeThreshold && filterCounter > 0) {
			filterCounter++;
		} else if (diff > derivativeThreshold) {
			filterCounter = 0;
		}
	}

	protected void cornerLocalization() {
		detectSingleLine = true;
		// Set the acceleration of both motor
		leftMotor.stop();
		leftMotor.setAcceleration(ZiplineLab.ACCELERATION);
		rightMotor.stop();
		rightMotor.setAcceleration(ZiplineLab.ACCELERATION);

		// Adjust the robot position, before it starts to rotate to ensure that
		// the light sensor will cross 4 black lines
		adjustRobotStartingPosition();

		// set the robot wheel's rotation speed to both motors
		leftMotor.setSpeed(ZiplineLab.ROTATIONSPEED);
		rightMotor.setSpeed(ZiplineLab.ROTATIONSPEED);

		detectFourLines = true;
		
		// rotate the robot 360 degrees
		leftMotor.rotate(convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 360), true);
		rightMotor.rotate(-convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 360), false);
		
		detectFourLines = false;
		
		correctOdometer();
		nav.travelTo(0, 0);
		nav.turnTo(-(odometer.getTheta() + (8 * Math.PI / 180)));
	}

	private void secondLocalization() {
		double theta = odometer.getTheta();
		lineCounter = 0;

		// fetching the values from the color sensor
		colorSensorValue.fetchSample(colorSensorData, 0);

		// getting the value returned from the sensor, and multiply it by
		// 1000 to scale
		float value = colorSensorData[0] * 1000;

		// computing the derivative at each point
		float diff = value - oldValue;

		// storing the current value, to be able to get the derivative on
		// the next iteration
		oldValue = value;
		if (diff < derivativeThreshold && filterCounter == 0) {
			Sound.beep();
			filterCounter++;
			lineCounter++;

			if (lineCounter == 1) {
				yminus = odometer.getTheta();

			} else if (lineCounter == 2) {
				xminus = odometer.getTheta();

			} else if (lineCounter == 3) {
				yplus = odometer.getTheta();

			} else if (lineCounter == 4) {
				xplus = odometer.getTheta();
			}

		} else if (diff < derivativeThreshold && filterCounter > 0) {
			filterCounter++;
		} else if (diff > derivativeThreshold) {
			filterCounter = 0;
		}
	}

	/**
	 * This method calculates the robot's actual x and y position using the angle
	 * values determine earlier using trigonometry, and then determine the theta by
	 * which the odometer is off. Then, it updates the x, y and theta values of the
	 * odometer in order to fix them. Then, it uses the navigation travelTo method
	 * to travel to the inputted destination.
	 * 
	 * @param xdestination
	 * @param ydestination
	 */
	private void correctOdometer() {
		System.out.println("X- : " + xminus);
		System.out.println("Y- : " + yminus);
		System.out.println("X+ : " + xplus);
		System.out.println("Y+ : " + yplus);
		
		thetay = yminus - yplus;
		thetax = xplus - xminus;

		this.x = -ZiplineLab.BOT_LENGTH * Math.cos(thetay / 2.0);
		this.y = -ZiplineLab.BOT_LENGTH * Math.cos(thetax / 2.0);
		deltaThetaY = (Math.PI / 2.0) - yminus + Math.PI + (thetay / 2.0);

		odometer.setX(this.x);
		odometer.setY(this.y);
		odometer.setTheta(odometer.getTheta() + deltaThetaY);

	}

	public void travelTo(double x, double y) {

		// get the current position and rotation of the robot, based on its
		// starting point
		currX = odometer.getX();
		currY = odometer.getY();
		currTheta = odometer.getTheta();

		// calculating the information needed (destination - current) for both y
		// and x, in order to calculate the minimum angle using arctan
		double deltaX = (x * ZiplineLab.TILE_LENGTH) - currX;
		double deltaY = (y * ZiplineLab.TILE_LENGTH) - currY;

		// calculating the minimum angle using Math.atan2 method
		double deltaTheta = Math.atan2(deltaX, deltaY) - currTheta;

		// rotate the robot towards its new way point
		turnTo(deltaTheta);

		// calculate the distance to next point using the built in pythagore
		// theorem
		// TODO try Math.Pyth
		double distToTravel = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);
		distToTravel = Math.sqrt(distToTravel);

		// travel to the next point, and don't wait until the action is
		// complete. So the boolean in both rotate method should be true
		leftMotor.setSpeed(ZiplineLab.FORWARDSPEED);
		rightMotor.setSpeed(ZiplineLab.FORWARDSPEED);

		rightMotor.rotate(convertDistance(ZiplineLab.WHEEL_RADIUS, distToTravel), true);
		leftMotor.rotate(convertDistance(ZiplineLab.WHEEL_RADIUS, distToTravel), false);

		leftMotor.stop(true);
		rightMotor.stop(true);

	}

	public void turnTo(double theta) {
		leftMotor.setSpeed(ZiplineLab.ROTATIONSPEED);
		rightMotor.setSpeed(ZiplineLab.ROTATIONSPEED);

		// adjusting the angle in order to have an optimal turn (a turn with the
		// minimum angle)
		if (theta <= -Math.PI) {
			theta += Math.PI * 2;
		} else if (theta > Math.PI) {
			theta -= Math.PI * 2;
		}

		theta = theta * 180.0 / Math.PI;

		// turn to the left if angle is negative
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, -theta - (20 * Math.PI / 180.0)),
					true);
			rightMotor.rotate(convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, -theta - (20 * Math.PI / 180.0)),
					false);
		}
		// turn to the right if angle is positive
		else {
			leftMotor.rotate(convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, theta - (10 * Math.PI / 180.0)),
					true);
			rightMotor.rotate(-convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, theta - (10 * Math.PI / 180.0)),
					false);
		}

	}

	/**
	 * This method adjusts the robot position before it starts rotating. It ensures
	 * that the light sensor, mounted on the back of the robot, runs over 4 black
	 * lines. It order to so, the robot, facing north after the ultrasonic
	 * localization, moves forward until it detects a line, and then move backwards
	 * 1.5 times its center distance. This, ensures that the robot is close enough
	 * to the horizontal black line facing the robot. Then, the method makes the
	 * robot turn 90 degrees and repeat the same procedure, so it can be placed
	 * close enough to the vertical line. Now the robot it close enough to the first
	 * intersection.
	 */
	private void adjustRobotStartingPosition() {
		// Set the wheel's rotation speed to ROTATESPEED
		leftMotor.setSpeed(ZiplineLab.ROTATIONSPEED);
		rightMotor.setSpeed(ZiplineLab.ROTATIONSPEED);

		// Rotate the robot by 45 degrees
		leftMotor.rotate(convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 45), true);
		rightMotor.rotate(-convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 45), false);

		while(detectSingleLine) {
			leftMotor.forward();
			rightMotor.forward();
		}
		
		leftMotor.stop(true);
		rightMotor.stop(true);
		
		// Move the robot backwards 1.5 * its center distance
		rightMotor.rotate(-convertDistance(ZiplineLab.WHEEL_RADIUS, 1.15 * ZiplineLab.BOT_LENGTH), true);
		leftMotor.rotate(-convertDistance(ZiplineLab.WHEEL_RADIUS, 1.15 * ZiplineLab.BOT_LENGTH), false);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}
}
