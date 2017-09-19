package main;

import java.util.ArrayList;
import java.util.List;

import org.joml.Vector3f;

public class Trajectory {

	private ArrayList<Segment> baseSegments, armSegments;
	
	// any specific info for a trajectory
	private boolean isFeasible;
	private boolean isReached;
	private float fitnessScore;
	private int current_index;
	
	private static float baseMass = 10, changeMass = (float) (baseMass + (Math.random() * 10));
	
	private float max_energy;
	private float energyTotal;
	
	public Trajectory(int n) {
		baseSegments = new ArrayList<>(n);
		armSegments = new ArrayList<>(n);
		current_index = 0;
		max_energy = Float.MAX_VALUE;
		energyTotal = 1;
	}

	public Trajectory(Trajectory trajectory) {
		this.armSegments = new ArrayList<>();
		this.baseSegments = new ArrayList<>();
		for(int i = 0; i<=trajectory.getSize(); i++) {
			this.armSegments.add(new Segment(trajectory.getArmPose(i)));
			this.baseSegments.add(new Segment(trajectory.getBasePose(i)));
		}
		this.isFeasible = trajectory.isFeasible;
		this.isReached = trajectory.isReached;
		this.fitnessScore = trajectory.fitnessScore;
		this.current_index = trajectory.current_index;
		this.max_energy = trajectory.max_energy;
		this.energyTotal = trajectory.energyTotal;
	}
	
	public void addBaseSegment(Segment segment) {
		this.baseSegments.add(segment);
	}
	
	public void addArmSegment(Segment segment) {
		this.armSegments.add(segment);
	}
	
	public void computeTrajectory() {
		Vector3f velocity;
		float energy = Float.MAX_VALUE;
		Vector3f slope1, slope2;
		int size = baseSegments.size();
		baseSegments.get(0).setStartVelocity(new Vector3f());
		armSegments.get(0).setStartVelocity(new Vector3f());
		for(int i=0;i<size-1;i++) {
			slope1 = baseSegments.get(i).getSlope();
			slope2 = baseSegments.get(i+1).getSlope();
			velocity = new Vector3f();
			if(!checkSign(slope1.x, slope2.x))
				velocity.x = (slope1.x + slope2.x) / 2;
			if(!checkSign(slope1.y, slope2.y))
				velocity.y = (slope1.y + slope2.y) / 2;
			if(!checkSign(slope1.x, slope2.x))
				velocity.z = (slope1.z + slope2.z) / 2;
			baseSegments.get(i).setEndVelocity(velocity);
			baseSegments.get(i).computeCoefficients((i+1) * 3);
			baseSegments.get(i+1).setStartVelocity(velocity);
		
			slope1 = armSegments.get(i).getSlope();
			slope2 = armSegments.get(i+1).getSlope();
			velocity = new Vector3f();
			if(!checkSign(slope1.x, slope2.x))
				velocity.x = (slope1.x + slope2.x) / 2;
			if(!checkSign(slope1.y, slope2.y))
				velocity.y = (slope1.y + slope2.y) / 2;
			if(!checkSign(slope1.x, slope2.x))
				velocity.z = (slope1.z + slope2.z) / 2;
			armSegments.get(i).setEndVelocity(velocity);
			armSegments.get(i).computeCoefficients((i+1) * 3);
			armSegments.get(i+1).setStartVelocity(velocity);
			
			energy = baseSegments.get(i).computeEnergy(true) + armSegments.get(i).computeEnergy(false);
			if(Float.compare(max_energy, energy) > 0)
				max_energy = energy;
		}

		baseSegments.get(size-1).setEndVelocity(new Vector3f());
		baseSegments.get(size-1).computeCoefficients(size * 3);
		
		armSegments.get(size-1).setEndVelocity(new Vector3f());
		armSegments.get(size-1).computeCoefficients(size * 3);
		
		energy = baseSegments.get(size-1).computeEnergy(true) + armSegments.get(size-1).computeEnergy(false);
		energyTotal += energy;
		if(Float.compare(max_energy, energy) > 0)
			max_energy = energy;
		
	}
	
	public List<Vector3f> execute(float time) {
		List<Vector3f> poses = new ArrayList<>();
		int size = baseSegments.size();
		if(current_index >= size) {
			poses.add(baseSegments.get(size-1).execute(baseSegments.get(size-1).getFinishingTime()));
			poses.add(armSegments.get(size-1).execute(armSegments.get(size-1).getFinishingTime()));
		} else {
			poses.add(baseSegments.get(current_index).execute(time));
			poses.add(armSegments.get(current_index).execute(time));
		}
		return poses;
	}
	
	public boolean isReached() {
		return isReached;
	}

	public void setReached(boolean isReached) {
		this.isReached = isReached;
	}

	public float getEnergyTotal() {
		return energyTotal;
	}

	public static float getBaseMass() {
		return baseMass;
	}

	public static float getChangeMass() {
		return changeMass;
	}

	public float getMaxEnergy() {
		return max_energy;
	}
	
	public int getCurrent_index() {
		return current_index;
	}
	
	public void setCurrent_index(float time) {
		current_index = (int) Math.floor(time / 3);
	}
	
	public boolean isFeasible() {
		return isFeasible;
	}

	public void setFeasible(boolean isFeasible) {
		this.isFeasible = isFeasible;
	}
	
	public float getFitnessScore() {
		return fitnessScore;
	}

	public void setFitnessScore(float fitnessScore) {
		this.fitnessScore = fitnessScore;
	}

	public boolean checkSign(float val1, float val2) {
		if((val1 < 0 && val2 > 0) || (val1>0 && val2 < 0))
			return true;
		return false;
	}
	
	public int getSize() {
		return baseSegments.size() - 1;
	}
	
	public Segment getBasePose(int index) {
		return baseSegments.get(index);
	}
	
	public Segment getArmPose(int index) {
		return armSegments.get(index);
	}

	public List<Segment> getArmPoses() {
		return armSegments;
	}
	
	public List<Segment> getArmSegments(int index) {
		int i = index;
		List<Segment> segments = new ArrayList<>();
		while(index != armSegments.size()) {
			segments.add(armSegments.get(i));
			armSegments.remove(i);
		}
		return segments;
	}
	
	public List<Segment> getBaseSegments(int index) {
		int i = index;
		List<Segment> segments = new ArrayList<>();
		while(index != baseSegments.size()) {
			segments.add(baseSegments.get(i));
			baseSegments.remove(i);
		}
		return segments;
	}
	
	public void addArmSegment(int i, Segment segment) {
		armSegments.add(i, segment);
	}
	
	public void addBaseSegment(int i, Segment segment) {
		baseSegments.add(i, segment);
	}
	
	public void addArmSegments(List<Segment> segments) {
		if(armSegments.size() == 0) {
			armSegments.addAll(segments);
			return;
		}
		segments.get(0).setStartPose(armSegments.get(armSegments.size()-1).getEndPose());
		armSegments.addAll(segments);
	}
	
	public void addBaseSegments(List<Segment> segments) {
		if(baseSegments.size() == 0) {
			baseSegments.addAll(segments);
			return;
		}
		segments.get(0).setStartPose(baseSegments.get(baseSegments.size()-1).getEndPose());
		baseSegments.addAll(segments);
	}
	
	public void removeArmPose(Segment segment) {
		armSegments.remove(segment);
	}
	
	public void removeBasePose(Segment segment) {
		baseSegments.remove(segment);
	}
	
}
