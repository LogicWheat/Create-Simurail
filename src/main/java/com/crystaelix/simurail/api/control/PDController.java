package com.crystaelix.simurail.api.control;


public class PDController implements FeedbackController {

	private double frequency;
	private double dampingRate;

	private double gain = 0;

	public PDController() {
	}

	public PDController(double frequency, double dampingRate) {
		this.frequency = Math.abs(frequency);
		this.dampingRate = Math.abs(dampingRate);
	}

	@Override
	public void setFrequency(double frequency) {
		this.frequency = Math.abs(frequency);
	}

	@Override
	public void setDampingRate(double dampingRate) {
		this.dampingRate = Math.abs(dampingRate);
	}

	@Override
	public double updateGain(double offset, double velocity, double maxGain, double timeStep) {
		if(timeStep <= 0) {
			return gain = 0;
		}
		double stiffness = frequency * frequency;
		double damping = 2 * frequency * dampingRate;
		double numerator = stiffness * offset - damping * velocity;
		double denominator = 1 + damping * timeStep + stiffness * timeStep * timeStep;
		return gain = Math.clamp(numerator / denominator, -Math.abs(maxGain), Math.abs(maxGain));
	}

	@Override
	public double getGain() {
		return gain;
	}

	@Override
	public void reset() {
		gain = 0;
	}
}
