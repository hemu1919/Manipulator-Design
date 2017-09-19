package main;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.joml.Vector2f;
import org.joml.Vector3f;
import org.lwjgl.glfw.GLFW;

import utils.DisplayManager;

public class RAMPImplementer {

	private final static float CONTROL_CYCLE = 0.5f;
	private final static float SENSING_CYCLE = 0.5f;
	private final static float freq = 1000;
	private final static int K = 10;
	private final static float D = 3;
	private final static float Q = 1e+4f;
	private final static float KNOTS_SIZE = 10;
	private final static String[] OPERATORS = new String[]{"insert", "delete", "crossover", "change", "swap", "stop"};
	
	private static float time_interval = 1;
	private static float control_counter = 0;
	private static float sensing_counter = 0;
	private static float execution_timer = 0;
	
	private Vector3f currentARMPose, goalARMPose, currentBASEPose, goalBASEPose;
	private ArrayList<Trajectory> population;
	private ArrayList<ArrayList<Integer>> subPopulation;
	private int N;
	private float gamma;
	private Trajectory selectedTraj;
	private float timer = 0;
	
	private ArrayList<Entity> environment;
	private static int count = 0;
	
	public RAMPImplementer(ArrayList<Entity> environment) {
		this.environment = environment;
	}
	
	public boolean isFinished() {
		return selectedTraj.getCurrent_index() >= (selectedTraj.getSize() + 1);
	}
	
	private void setPoses(Vector3f currentARMPose, Vector3f currentBASEPose, Vector3f goalPosition, Vector3f goalRotation) {
		this.currentARMPose = currentARMPose;
		this.currentBASEPose = currentBASEPose;
		float radius = Float.compare(RobotSpecs.getL2(), RobotSpecs.getL3()) < 0 ? RobotSpecs.getL3() : RobotSpecs.getL2();
		ArrayList<Vector2f> limits = new ArrayList<>();
		limits.add(new Vector2f(-180, 180));
		List<Float> thetas = genRandomn(limits);
		if(goalBASEPose == null)
			goalBASEPose = new Vector3f();
		goalBASEPose.x = (float) (goalPosition.x - radius * Math.cos(Math.toRadians(thetas.get(0))));
		goalBASEPose.z = (float) (goalPosition.z - radius * Math.sin(Math.toRadians(thetas.get(0))));
//		goalBASEPose = new Vector3f();
		this.goalARMPose = RobotSpecs.getThetaSpecs(new Vector3f(goalPosition.x - goalBASEPose.x, goalPosition.y, goalPosition.z - goalBASEPose.z), goalRotation);
		this.population = new ArrayList<>(N);
	}
	
	private void computeN(float L) {
		gamma = (float) Math.atan(L / D);
		int M = (int) Math.round(360 / gamma);
		this.N = K * M;
	}
	
	public void initializePlanner(float L,Vector3f currentARMPose, Vector3f currentBASEPose, Vector3f goalPosition, Vector3f goalRotation) {
		this.computeN(L);
		this.subPopulation = new ArrayList<>(N / K);
		this.setPoses(currentARMPose, currentBASEPose, goalPosition, goalRotation);
		selectedTraj = null;
		intitializePopulation();
		evaluatePopulation();
		getBestFit();
		createSubPopulations();
//		control_counter = (float) GLFW.glfwGetTime();
//		sensing_counter = (float) GLFW.glfwGetTime();
		control_counter = 0;
		sensing_counter = 0;
//		execution_timer = (float) GLFW.glfwGetTime();
		execution_timer = 0;
	}
	
	private synchronized void getBestFit() {
		Collections.sort(population, new Comparator<Trajectory>() {

			@Override
			public int compare(Trajectory o1, Trajectory o2) {
				return Double.compare(o1.getFitnessScore(), o2.getFitnessScore());
			}
			
		});
//		float time = (float) (GLFW.glfwGetTime() - execution_timer);
		float time = execution_timer;
		Trajectory trajectory = population.get(0);
		if(selectedTraj != null) {
			System.out.println(execution_timer);
			List<Vector3f> poses = selectedTraj.execute(time);
			trajectory.setCurrent_index(time);
			trajectory.getArmPose(trajectory.getCurrent_index()).setStartPose(poses.get(1));
			trajectory.getBasePose(trajectory.getCurrent_index()).setStartPose(poses.get(0));
			trajectory.computeTrajectory();
		}
		selectedTraj = trajectory;
	}

	public List<Vector3f> executePlanner() {
		Thread thread = new Thread(new Runnable() {
			
			@Override
			public void run() {
				synchronized (AnimationApp.getPlanner()) {
					modify();
					control_counter += 1 / freq;
					sensing_counter += 1 / freq;
					if(Float.compare(control_counter, 1 / CONTROL_CYCLE) >= 0) {
						createSubPopulations();
						evaluatePopulation();
						getBestFit();
						control_counter = 0;
//						control_counter = (float) GLFW.glfwGetTime();
					}
					if(Float.compare(sensing_counter, 1 / SENSING_CYCLE) >= 0) {
						evaluatePopulation();
						getBestFit();
						sensing_counter = 0;
//						sensing_counter = (float) GLFW.glfwGetTime();
					}
				}
			}
		});
		thread.start();
		// stopping condition
		synchronized (AnimationApp.getPlanner()) {
			execution_timer += 1 / freq;
			float time = execution_timer;
			selectedTraj.setCurrent_index(time);
			List<Vector3f> poseInfo = selectedTraj.execute(time);
			if(!isReached(poseInfo.get(1), poseInfo.get(0)))
				return poseInfo;
			else {
				execution_timer -= 1 / freq;
				return selectedTraj.execute(execution_timer);
			}
		}
	}
	
	private synchronized void modify() {
		int operator = (int) Math.round(Math.random() * (OPERATORS.length - 1));
		int count = (int) (Math.round(Math.random()) + 1);
		int index = -1;
		List<Float> values = genRandomn(count, new Vector2f(0, population.size() - 1));
		synchronized (AnimationApp.getPlanner()) {
			Collections.sort(population, new Comparator<Trajectory>() {

				@Override
				public int compare(Trajectory o1, Trajectory o2) {
					// TODO Auto-generated method stub
					return Double.compare(o1.getFitnessScore(), o2.getFitnessScore());
				}
			});
		}
		float max_fitness = population.get(population.size()-1).getFitnessScore();
		List<Trajectory> trajectories = apply(OPERATORS[operator], values);
		for(Trajectory trajectory : trajectories) {
			float fitness = trajectory.getFitnessScore();
			evaluateTrajectory(trajectory);
			if(Double.compare(fitness, max_fitness) < 0) {
				if(trajectory.isFeasible()) {
					do {
						index = (int) Math.round(Math.random()  * (population.size() - 1));
					} while(Double.compare(population.get(index).getFitnessScore(), selectedTraj.getFitnessScore()) == 0);
					population.set(index, trajectory);
				} else {
					do {
						index = (int) Math.round(Math.random()  * (population.size() - 1));
					} while(Double.compare(population.get(index).getFitnessScore(), selectedTraj.getFitnessScore()) == 0 || population.get(index).isFeasible());
					population.set(index, trajectory);
				}
			}
		}
	}

	private synchronized List<Trajectory> apply(String string, List<Float> values) {
//		Vector2f limit1 = new Vector2f(currentARMPose.x + 1, goalARMPose.x - 1);
//		Vector2f limit2 = new Vector2f(currentARMPose.y + 1, goalARMPose.y - 1);
//		Vector2f limit3 = new Vector2f(currentARMPose.z + 1, goalARMPose.z - 1);
		Vector2f limit1 = new Vector2f(-180, 180);
		Vector2f limit2 = new Vector2f(-150, 150);
		Vector2f limit3 = new Vector2f(-100, 100);
		Vector2f limit4 = new Vector2f(currentBASEPose.x, goalBASEPose.x);
		Vector2f limit5 = new Vector2f(currentBASEPose.z, goalBASEPose.z);
//		Vector2f limit4 = new Vector2f(-10, 10);
//		Vector2f limit5 = new Vector2f(-10, 10);
		Vector3f vector;
		List<Vector2f> limits = new ArrayList<>();
		List<Float> values1;
		List<Trajectory> trajectories = new ArrayList<>(values.size());
		int index1 = -1, index2 = -1;
		Trajectory trajectory;
		Segment segment1, segment2;
		if(string.equals("crossover")) {
			if(values.size() == 2) {
				float value = values.get(0);
				trajectory = new Trajectory(population.get((int) value));
				value = values.get(1);
				Trajectory trajectory2 = new Trajectory(population.get((int) value));
				if((trajectory.getCurrent_index() == trajectory.getSize()) || (trajectory2.getCurrent_index() == trajectory2.getSize()))
					return trajectories;
				index1 = (int) (trajectory.getCurrent_index() + Math.round(Math.random() * (trajectory.getSize() - trajectory.getCurrent_index() - 1)));
				index2 = (int) (trajectory2.getCurrent_index() + Math.round(Math.random() * (trajectory2.getSize() - trajectory2.getCurrent_index() - 1)));
				
				List<Segment> segments1 = trajectory.getArmSegments(index1);
				List<Segment> segments2 = trajectory2.getArmSegments(index2);
				trajectory.addArmSegments(segments2);
				trajectory2.addArmSegments(segments1);
				
				segments1 = trajectory.getBaseSegments(index1);
				segments2 = trajectory2.getBaseSegments(index2);
				trajectory.addBaseSegments(segments2);
				trajectory2.addBaseSegments(segments1);
				
				trajectory.computeTrajectory();
				trajectory2.computeTrajectory();
				trajectories.add(trajectory);
				trajectories.add(trajectory2);
			}
			return trajectories;
		}
		for(float value : values) {
			trajectory = new Trajectory(population.get((int) value));
			limits.clear();
			switch(string) {
			case "insert":
				index1 = (int) (trajectory.getCurrent_index() + Math.round(Math.random() * (trajectory.getSize() -  trajectory.getCurrent_index())));
				
				limits.add(limit1);
				limits.add(limit2);
				limits.add(limit3);
				values1 = genRandomn(limits);
				segment1 = trajectory.getArmPose(index1);
				vector = new Vector3f(values1.get(0), values1.get(1), values1.get(2));
				trajectory.addArmSegment(index1 + 1, new Segment(vector, segment1.getEndPose()));
				segment1.setEndPose(vector);
				limits.clear();
				
				limits.add(limit4);
				limits.add(limit5);
				values1 = genRandomn(limits);
				segment1 = trajectory.getBasePose(index1);
				vector = new Vector3f(values1.get(0), 0, values1.get(1));
				trajectory.addBaseSegment(index1 + 1, new Segment(vector, segment1.getEndPose()));
				segment1.setEndPose(vector);
				
				break;
			case "delete":
				if(trajectory.getCurrent_index() == trajectory.getSize())
					return trajectories;
				index1 = (int) (trajectory.getCurrent_index() + Math.round(Math.random() * (trajectory.getSize() - trajectory.getCurrent_index() - 1)));
				
				segment1 = trajectory.getArmPose(index1 + 1);
				trajectory.getArmPose(index1).setEndPose(segment1.getEndPose());
				trajectory.removeArmPose(segment1);
				
				segment1 = trajectory.getBasePose(index1 + 1);
				trajectory.getBasePose(index1).setEndPose(segment1.getEndPose());
				trajectory.removeBasePose(segment1);
				
				break;
			case "change":
				if(trajectory.getCurrent_index() == trajectory.getSize())
					return trajectories;
				index1 = (int) (trajectory.getCurrent_index() + Math.round(Math.random() * (trajectory.getSize() - trajectory.getCurrent_index() - 1)));
				
				limits.add(limit1);
				limits.add(limit2);
				limits.add(limit3);
				values1 = genRandomn(limits);
				vector = new Vector3f(values1.get(0), values1.get(1), values1.get(2));
				segment1 = trajectory.getArmPose(index1);
				segment1.setEndPose(vector);
				segment1 = trajectory.getArmPose(index1+1);
				segment1.setStartPose(vector);
				limits.clear();
				
				limits.add(limit4);
				limits.add(limit5);
				values1 = genRandomn(limits);
				vector = new Vector3f(values1.get(0), 0, values1.get(1));
				segment1 = trajectory.getBasePose(index1);
				segment1.setEndPose(vector);
				segment1 = trajectory.getBasePose(index1+1);
				segment1.setStartPose(vector);
				
				break;
			case "swap":
				if(trajectory.getCurrent_index() == trajectory.getSize())
					return trajectories;
				index1 = (int) (trajectory.getCurrent_index() + Math.round(Math.random() * (trajectory.getSize() - trajectory.getCurrent_index() - 1)));
				index2 = (int) (trajectory.getCurrent_index() + Math.round(Math.random() * (trajectory.getSize() - trajectory.getCurrent_index() - 1)));
				
				segment1 = trajectory.getArmPose(index1);
				segment2 = trajectory.getArmPose(index2);
				vector = segment1.getEndPose();
				segment1.setEndPose(segment2.getEndPose());
				segment1 = trajectory.getArmPose(index1 + 1);
				segment1.setStartPose(segment2.getEndPose());
				segment2.setEndPose(vector);
				segment2 = trajectory.getArmPose(index2 + 1);
				segment2.setStartPose(vector);
				
				segment1 = trajectory.getBasePose(index1);
				segment2 = trajectory.getBasePose(index2);
				vector = segment1.getEndPose();
				segment1.setEndPose(segment2.getEndPose());
				segment1 = trajectory.getBasePose(index1 + 1);
				segment1.setStartPose(segment2.getEndPose());
				segment2.setEndPose(vector);
				segment2 = trajectory.getBasePose(index2 + 1);
				segment2.setStartPose(vector);
				
				break;
			case "stop":
				break;
			}
			trajectory.computeTrajectory();
			trajectories.add(trajectory);
		}
		return trajectories;
	}

	private void intitializePopulation() {
		Vector2f limit1, limit2, limit3;
		limit1 = new Vector2f();
		limit2 = new Vector2f();
		limit3 = new Vector2f();
		List<Vector2f> limits = new ArrayList<>();
		List<Float> values;
		Vector3f startBASEPose, endBASEPose, startARMPose, endARMPose;
		for(int i=0;i<N;i++) {
			int n = (int) Math.round(Math.random() * KNOTS_SIZE) + 1;
			Trajectory trajectory = new Trajectory(n);
			startBASEPose = currentBASEPose;
			startARMPose = currentARMPose;
			for(int j=0;j<n;j++) {
				
				limit1.x = currentBASEPose.x;
				limit1.y = goalBASEPose.x;
				limit2.x = currentBASEPose.z;
				limit2.y = goalBASEPose.z;
//				limit1.x = -10;
//				limit1.y = 10;
//				limit2.x = -10;
//				limit2.y = 10;
				limits.add(limit1);
				limits.add(limit2);
				values = genRandomn(limits);
				endBASEPose = new Vector3f(values.get(0), 0, values.get(1));
				trajectory.addBaseSegment(new Segment(startBASEPose, endBASEPose));
				startBASEPose = endBASEPose;
				limits.clear();
				
//				limit1.x = currentARMPose.x + 1;
//				limit1.y = goalARMPose.x - 1;
//				limit2.x = currentARMPose.y + 1;
//				limit2.y = goalARMPose.y - 1;
//				limit3.x = currentARMPose.z + 1;
//				limit3.y = goalARMPose.z - 1;
				limit1.x = -180;
				limit1.y = 180;
				limit2.x = -150;
				limit2.y = 150;
				limit3.x = -100;
				limit3.y = 100;
				limits.add(limit1);
				limits.add(limit2);
				limits.add(limit3);
				values = genRandomn(limits);
				endARMPose = new Vector3f(values.get(0), values.get(1), values.get(2));
				trajectory.addArmSegment(new Segment(startARMPose, endARMPose));
				startARMPose = endARMPose;
				limits.clear();
				
			}
			trajectory.addBaseSegment(new Segment(startBASEPose, goalBASEPose));
			trajectory.addArmSegment(new Segment(startARMPose, goalARMPose));
			trajectory.computeTrajectory();
			population.add(trajectory);
		}
		
		Trajectory trajectory = new Trajectory(1);
		trajectory.addBaseSegment(new Segment(currentBASEPose, goalBASEPose));
		trajectory.addArmSegment(new Segment(currentARMPose, goalARMPose));
		trajectory.computeTrajectory();
		population.add(trajectory);
		
	}
	
	private List<Float> genRandomn(List<Vector2f> limits) {
		int dimen = limits.size();
		ArrayList<Float> values = new ArrayList<>(dimen);
		for(Vector2f limit : limits)
			values.add(limit.x + (float) (Math.random() * (limit.y - limit.x)));
		return values;
	}
	
	private List<Float> genRandomn(int dimen, Vector2f limit) {
		ArrayList<Float> values = new ArrayList<>(dimen);
		for(int i=0;i<dimen;i++)
			values.add(limit.x + (float) (Math.random() * (limit.y - limit.x)));
		return values;
	}
	
	private synchronized void evaluatePopulation() {
		for(Trajectory trajectory : population) {
			evaluateTrajectory(trajectory);
		}
	}
	
	private void evaluateTrajectory(Trajectory trajectory) {
		boolean isFeasible = feasibilityCheck(trajectory);
		trajectory.setFeasible(isFeasible);
		float cost = evaluateFeasible(trajectory);
		if(isFeasible)
			trajectory.setFitnessScore(cost);
		else
			trajectory.setFitnessScore(cost + evaluateInFeasible());
	}
	
	private float evaluateInFeasible() {
		return Q / time_interval;
	}

	private float evaluateFeasible(Trajectory trajectory) {
		float a2 = KNOTS_SIZE + 1, c2 = 0.6f, c1 = 0.01f, a1 = trajectory.getMaxEnergy();
		float T = c2 * (trajectory.getSize() - trajectory.getCurrent_index() + 1) * trajectory.getBasePose(0).getBalanceTime() / a2;
		float E = c1 * trajectory.getEnergyTotal() / a1;
		return E + T;
	}

	private boolean feasibilityCheck(Trajectory trajectory) {
		Vector2f limit = new Vector2f(trajectory.getCurrent_index(), trajectory.getSize());
		int n = (int) Math.round(Math.random() * (trajectory.getSize() - trajectory.getCurrent_index() + 1));
		List<Float> values = genRandomn(n, limit);
		for(float value : values) {
			Segment base = trajectory.getBasePose((int) value), arm = trajectory.getArmPose((int) value);
			List<Float> values1 = genRandomn(100, new Vector2f(base.getStartingTime(), base.getFinishingTime()));
			Collections.sort(values1);
			for(float value1 : values1) {
				float t = value1;
				Vector3f basePose = base.execute(t);
				//t = arm.getStartingTime() + (float) (Math.random() * arm.getBalanceTime());
				List<Vector3f> positions = RobotSpecs.getPosition(arm.execute(t), basePose);
				for(Vector3f position : positions) {
					for(Entity entity : environment) {
						float range = position.distance(entity.getPosition());
						float range1 = basePose.distance(entity.getPosition());
						if((range - (2 * RobotSpecs.getCOLLISION_RANGE()) < D) || (range1 - (2 * RobotSpecs.getCOLLISION_RANGE()) < D)) {
							trajectory.setReached(true);
							time_interval = t;
							return false;
						}
					}
				}
			}
			
		}
		
		trajectory.setReached(false);
		return true;
	}
	
	private boolean isReached(Vector3f armPose, Vector3f basePose) {
		List<Vector3f> positions = RobotSpecs.getPosition(armPose, basePose);
		for(Vector3f position : positions) {
			for(Entity entity : environment) {
				float range = distance(position, entity.getPosition());
				float range1 = distance(position, entity.getPosition());
				if((range - (2 * RobotSpecs.getCOLLISION_RANGE()) < D) || (range1 - (2 * RobotSpecs.getCOLLISION_RANGE()) < D))
					return true;
			}
		}
		return false;
	}
	
	private float distance(Vector3f vector1, Vector3f vector2) {
		return (float) Math.sqrt(Math.pow(vector1.x - vector2.x, 2) + Math.pow(vector1.y - vector2.y, 2) + Math.pow(vector1.z - vector2.z, 2));
	}
	
	private void createSubPopulations() {
		Vector3f reference = new Vector3f();
		clearSubPopulations();
		for(int i=0;i<N;i++) {
			Trajectory trajectory = population.get(i);
			Segment segment = trajectory.getArmPose(trajectory.getCurrent_index());
			float phi = (float) Math.toDegrees(Math.acos(reference.dot(getDifference(segment.getStartPose(), segment.getEndPose())) / (getMagnitude(segment.getStartPose()) * getMagnitude(segment.getEndPose()))));
			int index = (int) Math.floor(phi / gamma);
			ArrayList<Integer> indices = subPopulation.get(index);
			if(indices == null)
				indices = new ArrayList<>();
			indices.add(i);
		}
	}
	
	private void clearSubPopulations() {
		if(subPopulation.size() == 0)
			for(int i=0;i<N/K;i++)
				subPopulation.add(new ArrayList<>());
		else
			for(List<Integer> list : subPopulation)
				list.clear();
	}

	private Vector3f getDifference(Vector3f vector1, Vector3f vector2) {
		Vector3f diff = new Vector3f(vector1.x - vector2.x, vector1.y - vector2.y, vector1.z - vector2.z);
		return diff;
	}
	
	private float getMagnitude(Vector3f vector) {
		return (float) Math.sqrt((vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z));
	}
	
}
