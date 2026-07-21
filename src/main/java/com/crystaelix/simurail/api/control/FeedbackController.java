package com.crystaelix.simurail.api.control;


public interface FeedbackController {

	void setFrequency(double frequency);

	void setDampingRate(double dampingRate);

	double updateGain(double offset, double velocity, double maxGain, double timeStep);

	double getGain();
	
	void reset();
}
