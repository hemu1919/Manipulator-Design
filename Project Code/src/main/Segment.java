package main;

import java.util.ArrayList;

import org.joml.Vector3f;
import org.joml.Vector4f;

public class Segment {

	private Vector3f startPose, endPose;
	private Vector3f startVelocity, endVelocity;
	private float time, energy;
	private int index;
	
	// any other segment specific info
	private ArrayList<Vector4f> coefficients;
	private boolean isFeasible;
	
	public Segment(Vector3f startPose, Vector3f endPose) {
		this.startPose = startPose;
		this.endPose = endPose;
		this.time = 3;
		this.coefficients = new ArrayList<>(3);
		this.coefficients.add(new Vector4f());
		this.coefficients.add(new Vector4f());
		this.coefficients.add(new Vector4f());
		this.coefficients.trimToSize();
		this.isFeasible = true;
		this.energy = 0;
	}
	
	public Segment(Segment segment) {
		this.startPose = segment.startPose;
		this.endPose = segment.endPose;
		this.startVelocity = segment.startVelocity;
		this.endVelocity = segment.endVelocity;
		this.time = segment.time;
		this.energy = segment.energy;
		this.index = segment.index;
		this.coefficients = new ArrayList<>();
		for(Vector4f vector : segment.coefficients)
			this.coefficients.add(new Vector4f(vector));
		this.isFeasible = segment.isFeasible;
	}
	
	public float getEnergy() {
		return energy;
	}

	public void setEnergy(float energy) {
		this.energy = energy;
	}

	public boolean isFeasible() {
		return isFeasible;
	}

	public void setFeasible(boolean isFeasible) {
		this.isFeasible = isFeasible;
	}

	public void setStartPose(Vector3f startPose) {
		this.startPose = startPose;
	}

	public void setEndPose(Vector3f endPose) {
		this.endPose = endPose;
	}

	public void setStartVelocity(Vector3f startVelocity) {
		this.startVelocity = startVelocity;
	}

	public void setEndVelocity(Vector3f endVelocity) {
		this.endVelocity = endVelocity;
	}

	public void setTime(float time) {
		this.time = time;
	}

	public void computeCoefficients(float time) {
		Vector4f coefficient;
		this.index = (int) (time / this.time);
		this.coefficients.clear();
		
		coefficient = new Vector4f();
		coefficient.x = startPose.x;
		coefficient.y = startVelocity.x;
		coefficient.z = ((endPose.x - startPose.x) * 3  / (time * time)) - (((2 * startVelocity.x) + endVelocity.x) / time);
		coefficient.w = ((startVelocity.x + endVelocity.x) / (time * time))- (((endPose.x - startPose.x) * 2) / (time * time * time));
		coefficients.add(coefficient);
		
		coefficient = new Vector4f();
		coefficient.x = startPose.y;
		coefficient.y = startVelocity.y;
		coefficient.z = ((endPose.y - startPose.y) * 3  / (time * time)) - (((2 * startVelocity.y) + endVelocity.y) / time);
		coefficient.w = ((startVelocity.y + endVelocity.y) / (time * time)) - (((endPose.y - startPose.y) * 2) / (time * time * time));
		coefficients.add(coefficient);
		
		coefficient = new Vector4f();
		coefficient.x = startPose.z;
		coefficient.y = startVelocity.z;
		coefficient.z = ((endPose.z - startPose.z) * 3  / (time * time)) - (((2 * startVelocity.z) + endVelocity.z) / time);
		coefficient.w = ((startVelocity.z + endVelocity.z) / (time * time))- (((endPose.z - startPose.z) * 2) / (time * time * time));
		coefficients.add(coefficient);
		
	}
	
	public float computeEnergy(boolean isBase) {
		Vector3f velocity = new Vector3f();
		velocity.x = Math.abs(endVelocity.x - startVelocity.x) / time;
		velocity.z = Math.abs(endVelocity.z - startVelocity.z) / time;
		if(isBase) {
			float effective_velocity = (float) (Math.pow(velocity.x, 2) + Math.pow(velocity.y, 2) + Math.pow(velocity.z, 2));
			energy = 3 * Trajectory.getBaseMass() * effective_velocity;
		} else {
			velocity.y = Math.abs(endVelocity.y- startVelocity.y) / time;
			energy = Trajectory.getBaseMass() * velocity.x;
			energy += Trajectory.getChangeMass() * velocity.y;
			energy += Trajectory.getChangeMass() * velocity.z;
			
		}
		return energy;
	}
	
	public Vector3f execute(float t) {
		Vector3f theta = new Vector3f();
		Vector4f coefficient;
		
		coefficient = coefficients.get(0);
		theta.x = coefficient.x + (coefficient.y * t) + (coefficient.z * t * t) + (coefficient.w * t * t * t);
		
		coefficient = coefficients.get(1);
		theta.y = coefficient.x + (coefficient.y * t) + (coefficient.z * t * t) + (coefficient.w * t * t * t);
		
		coefficient = coefficients.get(2);
		theta.z = coefficient.x + (coefficient.y * t) + (coefficient.z * t * t) + (coefficient.w * t * t * t);
		
		return theta;
	}
	
	public Vector3f getStartPose() {
		return startPose;
	}

	public Vector3f getEndPose() {
		return endPose;
	}

	public Vector3f getSlope() {
		Vector3f slope = new Vector3f();
		slope.x = (endPose.x - startPose.x) / time;
		slope.y = (endPose.y - startPose.y) / time;
		slope.z = (endPose.z - startPose.z) / time;
		return slope;
	}
	
	public float getBalanceTime() {
		return time;
	}
	
	public float getFinishingTime() {
		return index * time;
	}
	
	public float getStartingTime() {
		return (index-1) * time;
	}

	@Override
	public String toString() {
		return "Segment [startPose = "+startPose+ ", endPose = "+endPose+"]";
	}
	
}
