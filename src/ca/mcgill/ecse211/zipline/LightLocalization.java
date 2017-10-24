package ca.mcgill.ecse211.zipline;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalization extends Thread {

	private static final Port lightSampler = LocalEV3.get().getPort("S2");

	private SensorModes colosSamplerSensor = new EV3ColorSensor(lightSampler);
	private SampleProvider colorSensorValue = colosSamplerSensor.getMode("Red");

	private Navigation nav;

	private float[] colorSensorData = new float[colosSamplerSensor.sampleSize()];

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

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

	public LightLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			Navigation nav, int[] points) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.nav = nav;
		x0 = points[0];
		y0 = points[1];
		xC = points[2];
		yC = points[3];
		corner = points[4];
	}

	public void run() {
		Button.waitForAnyPress();
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

		// rotate the robot 360 degrees
		leftMotor.rotate(convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 360), true);
		rightMotor.rotate(-convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 360), true);

		// while rotating, get the angles which are used to correct the robot's
		// position and orientation
		determineLocalizationAngles();

		// turn and travel to (0, 0) which is the first intersection
		correctOdometer(0, 0);

		nav.travelTo(0, 0);

		// wait until the robot reached (0 ,0)
		// while ((rightMotor.isMoving() && leftMotor.isMoving()));

		// let the robot head north
		nav.turnTo(-(odometer.getTheta() + (8 * Math.PI / 180)));

		// once robot adjusts to its relative (0,0)
		// change the actual odometer x and y to what the board is supposed to be
		// EX. if at corner 1 the relative (0,0) is actually (7,1)
		if (corner == 0) {
			odometer.setX(ZiplineLab.TILE_LENGTH);
			odometer.setY(ZiplineLab.TILE_LENGTH);
		} else if (corner == 1) {
			odometer.setX(7 * ZiplineLab.TILE_LENGTH);
			odometer.setY(ZiplineLab.TILE_LENGTH);
			odometer.setTheta(3 * Math.PI / 2);
		} else if (corner == 2) {
			odometer.setX(7 * ZiplineLab.TILE_LENGTH);
			odometer.setY(7 * ZiplineLab.TILE_LENGTH);
			odometer.setTheta(Math.PI);
		} else if (corner == 3) {
			odometer.setX(ZiplineLab.TILE_LENGTH);
			odometer.setY(7 * ZiplineLab.TILE_LENGTH);
			odometer.setTheta(Math.PI / 2);
		}

		System.out.println("X0: " + x0 + " Y0: " + y0);

		Button.waitForAnyPress();

		nav.travelTo(x0, y0);

		// travelTo zipline coordinate and move forward on it
		Button.waitForAnyPress();

		leftMotor.rotate(convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 360), true);
		rightMotor.rotate(-convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 360), true);

		// while rotating, get the angles which are used to correct the robot's
		// position and orientation
		secondLocalization();

		// turn and travel to (0, 0) which is the first intersection
		correctOdometer(x0 * ZiplineLab.TILE_LENGTH, y0 * ZiplineLab.TILE_LENGTH);

		nav.travelTo(x0, y0);

		// wait until the robot reached (0 ,0)
		// while ((rightMotor.isMoving() && leftMotor.isMoving()));

		// let the robot head north
		nav.turnTo(-(odometer.getTheta()));

	}

	private void secondLocalization() {
		double theta = odometer.getTheta();
		lineCounter = 0;
		
		while (leftMotor.isMoving() && rightMotor.isMoving()) {
			
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
			// System.out.println(diff);
			System.out.println();
			if (diff < derivativeThreshold && filterCounter == 0) {
				
				if(theta >= 0*Math.PI/180 && theta <= 90*Math.PI/180) {
					if (lineCounter == 1) {
						xminus = odometer.getTheta();

					} else if (lineCounter == 2) {
						yplus = odometer.getTheta();

					} else if (lineCounter == 3) {
						xplus = odometer.getTheta();

					} else if (lineCounter == 4) {
						yminus = odometer.getTheta();
					}
				}
				
				else if(theta > 90*Math.PI/180 && theta <= 180*Math.PI/180) {
					if (lineCounter == 1) {
						yplus = odometer.getTheta();

					} else if (lineCounter == 2) {
						xplus = odometer.getTheta();

					} else if (lineCounter == 3) {
						yminus = odometer.getTheta();

					} else if (lineCounter == 4) {
						xminus = odometer.getTheta();
					}
				}
				
				else if(theta > 180*Math.PI/180 && theta <= 270*Math.PI/180) {
					if (lineCounter == 1) {
						xplus = odometer.getTheta();

					} else if (lineCounter == 2) {
						yminus = odometer.getTheta();

					} else if (lineCounter == 3) {
						xminus = odometer.getTheta();

					} else if (lineCounter == 4) {
						xplus = odometer.getTheta();
					}
				}
				
				else if((theta >270*Math.PI/180 && theta < 360*Math.PI/180)) {
					if (lineCounter == 1) {
						yminus = odometer.getTheta();

					} else if (lineCounter == 2) {
						xminus = odometer.getTheta();

					} else if (lineCounter == 3) {
						yplus = odometer.getTheta();

					} else if (lineCounter == 4) {
						xplus = odometer.getTheta();
					}
				}

			} else if (diff < derivativeThreshold && filterCounter > 0) {
				filterCounter++;
			} else if (diff > derivativeThreshold) {
				filterCounter = 0;
			}

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
	private void correctOdometer(double x, double y) {

		thetay = yminus - yplus;
		thetax = xplus - xminus;

		this.x = -ZiplineLab.BOT_LENGTH * Math.cos(thetay / 2.0) + x;
		this.y = -ZiplineLab.BOT_LENGTH * Math.cos(thetax / 2.0) + y;
		deltaThetaY = (Math.PI / 2.0) - yminus + Math.PI + (thetay / 2.0);

		odometer.setX(this.x);
		odometer.setY(this.y);
		odometer.setTheta(odometer.getTheta() + deltaThetaY);

	}

	/**
	 * while the robot is rotating on itself, get the measured values of the light
	 * sensor. A black line is detected when the derivative less than a threshold.
	 * When it is detected, a filter is added because sometimes, since the frequency
	 * of the value returned by the robot is high, the derivative can be less than
	 * the threshold for 2 or 3 consecutive returned values. This filter makes sure
	 * that we enter the if statement only the first time the derivative is less
	 * than threshold. Moreover, a line counter is added to determine which theta is
	 * return by the odometer (theta (x-), theta (y+), theta (x+), and theta (y-)
	 * respectively).
	 */
	private void determineLocalizationAngles() {
		lineCounter = 0;
		while (leftMotor.isMoving() && rightMotor.isMoving()) {
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
			// System.out.println(diff);
			System.out.println();
			if (diff < derivativeThreshold && filterCounter == 0) {
				Sound.beep();
				filterCounter++;
				lineCounter++;

				if (lineCounter == 1) {
					xminus = odometer.getTheta();

				} else if (lineCounter == 2) {
					yplus = odometer.getTheta();

				} else if (lineCounter == 3) {
					xplus = odometer.getTheta();

				} else if (lineCounter == 4) {
					yminus = odometer.getTheta();
				}
			} else if (diff < derivativeThreshold && filterCounter > 0) {
				filterCounter++;
			} else if (diff > derivativeThreshold) {
				filterCounter = 0;
			}

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

		// lol fuck all that, it's way to slow
		// implementing jeremy's method

		// Set the wheel's rotation speed to ROTATESPEED
		leftMotor.setSpeed(ZiplineLab.ROTATIONSPEED);
		rightMotor.setSpeed(ZiplineLab.ROTATIONSPEED);
		// Rotate the robot by 45 degrees
		leftMotor.rotate(convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 45), true);
		rightMotor.rotate(-convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 45), false);

		// drive forward until line is detected
		detectBlackLine();

		// This method gets the data from the light sensor when the robot is
		// moving forward, and returns when a black line is detected
		// detectBlackLine();
		// // TODO make this work in all corners
		// // odometer.setY(ZiplineLab.initialX);
		// odometer.setY(ZiplineLab.BOT_LENGTH); // set to distance btwn wheels and
		// sensor
		//
		// // rightMotor.stop();
		// // leftMotor.stop();
		//
		// Move the robot backwards 1.5 * its center distance
		rightMotor.rotate(-convertDistance(ZiplineLab.WHEEL_RADIUS, 1.15 * ZiplineLab.BOT_LENGTH), true);
		leftMotor.rotate(-convertDistance(ZiplineLab.WHEEL_RADIUS, 1.15 * ZiplineLab.BOT_LENGTH), false);
		//
		// // Set the wheel's rotation speed to ROTATESPEED
		// leftMotor.setSpeed(ZiplineLab.ROTATIONSPEED);
		// rightMotor.setSpeed(ZiplineLab.ROTATIONSPEED);
		//
		// // Rotate the robot by 90 degrees
		// leftMotor.rotate(convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK, 90),
		// true);
		// rightMotor.rotate(-convertAngle(ZiplineLab.WHEEL_RADIUS, ZiplineLab.TRACK,
		// 90), false);
		//
		// // Move forward, and return when a black line is detected
		// detectBlackLine();
		// // TODO make this work in all corners
		// // odometer.setX(ZiplineLab.initialY);
		// odometer.setX(ZiplineLab.BOT_LENGTH);// set to distance btwn wheels and
		// sensor

		// may have to stop motors here
		// rightMotor.stop();
		// leftMotor.stop();

		/*
		 * // Move the robot backwards 1.5 * its center distance
		 * leftMotor.rotate(-convertDistance(RADIUS, 1.5 * CENTERDISTANCE), true);
		 * rightMotor.rotate(-convertDistance(RADIUS, 1.5 * CENTERDISTANCE), false);
		 * 
		 * // Rotate the robot by 90 degrees leftMotor.setSpeed(ROTATESPEED);
		 * rightMotor.setSpeed(ROTATESPEED);
		 * 
		 * // Rotate the robot by -90 degrees leftMotor.rotate(-convertAngle(RADIUS,
		 * TRACK, 90), true); rightMotor.rotate(convertAngle(RADIUS, TRACK, 90), false);
		 */
	}

	/**
	 * This method makes the robot move forward. While moving, get the value
	 * returned by the light sensor, compute the derivative, and if the value of the
	 * derivative (diff) is than the threshold, a black line is detected. Beep and
	 * break out of the loop when the black line is detected.
	 */
	private void detectBlackLine() {

		leftMotor.setSpeed(ZiplineLab.FORWARDSPEED);
		rightMotor.setSpeed(ZiplineLab.FORWARDSPEED);

		leftMotor.forward();
		rightMotor.forward();

		while (leftMotor.isMoving() && rightMotor.isMoving()) {
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
			// System.out.println(diff);
			System.out.println();
			if (diff < derivativeThreshold) {
				Sound.beep();
				break;
			}

		}
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double TRACK, double angle) {
		return convertDistance(radius, Math.PI * TRACK * angle / 360.0);
	}
}
