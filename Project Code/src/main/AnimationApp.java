package main;

import java.util.ArrayList;
import java.util.List;

import org.joml.Vector3f;

import animatedModel.Joint;
import dataStructures.JointData;
import dataStructures.VertexSkinData;
import renderEngine.RenderEngine;
import scene.Scene;
import utils.DisplayManager;

public class AnimationApp {

	private static RAMPImplementer planner;
	
	/**
	 * Initialises the engine and loads the scene. For every frame it updates the
	 * camera, updates the animated entity (which updates the animation),
	 * renders the scene to the screen, and then updates the display. When the
	 * display is close the engine gets cleaned up.
	 * 
	 * @param args
	 */
		
	public static void main(String[] args) {

		RenderEngine engine = RenderEngine.init();

		//Joint joint1 = new Joint(, name, bindLocalTransform)
		ArrayList<Entity> environment = new ArrayList<>();
		
		Entity entity = new Entity(new Vector3f(-10, RobotSpecs.getL2() + 4, 0), GeneralSettings.RES_FOLDER);
		environment.add(entity);
		entity = new Entity(new Vector3f(-4, RobotSpecs.getL2() + 15, 0), GeneralSettings.RES_FOLDER);
		environment.add(entity);
		
		List<Vector3f> goalPoses;
		int index;
		
		goalPoses = new ArrayList<>();
		index = 0;
		goalPoses.add(new Vector3f(6.7132387f, 1.617275f, 0));
		goalPoses.add(new Vector3f(-16.7132387f, 11.617275f, 0));
		goalPoses.add(new Vector3f(26.7132387f, 11.617275f, 0));
		goalPoses.add(new Vector3f(-20.7132387f, 11.617275f, 0));
		
		planner = new RAMPImplementer(environment);
		Scene scene = SceneLoader.loadScene(GeneralSettings.RES_FOLDER, goalPoses.get(index));

		while (!DisplayManager.isCloseRequested()) {
			scene.getCamera().move();
			if(planner.isFinished() && ++index < goalPoses.size())
				scene.getAnimatedModel().doAnimation(goalPoses.get(index));
			scene.getAnimatedModel().update();
			engine.renderScene(scene, environment);
			engine.update();
		}

		engine.close();

	}

	public static RAMPImplementer getPlanner() {
		return planner;
	}

	public static void setPlanner(RAMPImplementer planner) {
		AnimationApp.planner = planner;
	}

}
