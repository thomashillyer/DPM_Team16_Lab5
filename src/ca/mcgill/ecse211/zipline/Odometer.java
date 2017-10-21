package ca.mcgill.ecse211.zipline;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x;
	private double y;
	private double theta;

	// previous tacho count
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;

	// instance of motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private static final long ODOMETER_PERIOD = 25; /* odometer update period, in ms */

	private Object lock; /* lock object for mutual exclusion */

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();

			// get value of tacho count for distance calculation
			int currTachoR = rightMotor.getTachoCount();
			int currTachoL = leftMotor.getTachoCount();

			// calculate distance of each wheel
			// dist = 2*PI*wheelRadius*wheelRotation/360
			double distR = Math.PI * ZiplineLab.WHEEL_RADIUS * (currTachoR - rightMotorTachoCount) / 180;
			double distL = Math.PI * ZiplineLab.WHEEL_RADIUS * (currTachoL - leftMotorTachoCount) / 180;
			// calculate change in theta and center disctance
			double deltaD = .5 * (distR + distL);
			double deltaT = (distL - distR) / ZiplineLab.TRACK;

			synchronized (lock) {
				// set all values in sync block to avoid race conditions
				rightMotorTachoCount = currTachoR;
				leftMotorTachoCount = currTachoL;

				theta += deltaT;

				// correction so theta is never negative
				if (theta > 2 * Math.PI)
					theta += -(2 * Math.PI);
				else if (theta < 0)
					theta += 2 * Math.PI;

				// calculate change in x and y
				double dX = deltaD * Math.sin(theta);
				double dY = deltaD * Math.cos(theta);

				x += dX;
				y += dY;
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount
	 *            the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount
	 *            the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;
		}
	}
}
