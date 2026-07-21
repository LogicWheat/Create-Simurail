package com.crystaelix.simurail.api.control;

import com.crystaelix.simurail.api.math.SimurailMath;

public class ADRController implements FeedbackController {

	private double frequency;
	private double dampingRate;
	private double observerFrequencyMult; 
	private double disturbanceDecayRate;

	private double lastTimeStep = 0;
	private double cachedDisturbanceDecay = -1;

	private double z1 = 0;
	private double z2 = 0;
	private double z3 = 0;
	private double gain = 0;

	public ADRController() {
	}

	public ADRController(double frequency, double dampingRate, double observerFrequencyMult, double disturbanceDecayRate) {
		this.frequency = Math.abs(frequency);
		this.dampingRate = Math.abs(dampingRate);
		this.observerFrequencyMult = Math.max(1, Math.abs(observerFrequencyMult));
		this.disturbanceDecayRate = Math.abs(disturbanceDecayRate);
	}

	@Override
	public void setFrequency(double frequency) {
		this.frequency = Math.abs(frequency);
	}

	@Override
	public void setDampingRate(double dampingRate) {
		this.dampingRate = Math.abs(dampingRate);
	}

	public void setObserverFrequencyMult(double observerFrequencyMult) {
		this.observerFrequencyMult = Math.max(1, Math.abs(observerFrequencyMult));
	}

	public void setDisturbanceDecayRate(double disturbanceDecayRate) {
		cachedDisturbanceDecay = -1;
		this.disturbanceDecayRate = Math.abs(disturbanceDecayRate);
	}

	private double getDisturbanceDecay(double timeStep) {
		if(cachedDisturbanceDecay < 0 || Math.abs(lastTimeStep - timeStep) > SimurailMath.EPSILON) {
			lastTimeStep = timeStep;
			cachedDisturbanceDecay = Math.exp(-timeStep * disturbanceDecayRate);
		}
		return cachedDisturbanceDecay;
	}

	@Override
	public double updateGain(double offset, double velocity, double maxGain, double timeStep) {
		if(timeStep <= 0) {
			reset();
			return 0;
		}
		z3 *= getDisturbanceDecay(timeStep);
		double y = -offset;
		double observerFrequency = frequency * observerFrequencyMult;
		double l1 = 3 * observerFrequency;
		double l2 = 3 * observerFrequency * observerFrequency;
		double l3 = observerFrequency * observerFrequency * observerFrequency;
		double dt2 = timeStep * timeStep;
		double dt3 = dt2 * timeStep;
		double delta = 1 + timeStep * l1 + dt2 * l2 + dt3 * l3;
		z1 = (z1 + timeStep * z2 + dt2 * z3 + dt2 * gain - y) / delta + y;
		z3 = z3 + timeStep * l3 * (y - z1);
		double maxDisturbance = Math.abs(maxGain) * 0.75;
		if(Math.abs(z3) > maxDisturbance) {
			z3 = Math.signum(z3) * maxDisturbance;
		}
		z2 = z2 + timeStep * gain + timeStep * z3 + timeStep * l2 * (y - z1);
		double stiffness = frequency * frequency;
		double damping = 2 * frequency * dampingRate;
		double numerator = stiffness * offset - damping * velocity;
		double denominator = 1 + damping * timeStep + stiffness * dt2;
		return gain = Math.clamp(numerator / denominator - z3, -Math.abs(maxGain), Math.abs(maxGain));
	}

	@Override
	public double getGain() {
		return gain;
	}

	@Override
	public void reset() {
		z1 = 0;
		z2 = 0;
		z3 = 0;
		gain = 0;
	}
}
