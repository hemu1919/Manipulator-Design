package animation;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.joml.AxisAngle4d;
import org.joml.AxisAngle4f;
import org.joml.Matrix3d;
import org.joml.Matrix4d;
//import org.lwjgl.util.vector.Matrix4f;
import org.joml.Matrix4f;
import org.joml.Matrix4fc;
import org.joml.Matrix4x3f;
import org.joml.Matrix4x3fc;
import org.joml.Quaterniond;
import org.joml.Quaternionf;
import org.joml.Quaternionfc;
import org.joml.Vector2fc;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import org.joml.Vector4f;
import org.joml.Vector4fc;

import com.sun.javafx.geom.Matrix3f;

import animatedModel.AnimatedModel;
import animatedModel.Joint;
import main.AnimationApp;
import main.RobotSpecs;
import utils.DisplayManager;

/**
 * 
 * This class contains all the functionality to apply an animation to an
 * animated entity. An Animator instance is associated with just one
 * {@link AnimatedModel}. It also keeps track of the running time (in seconds)
 * of the current animation, along with a reference to the currently playing
 * animation for the corresponding entity.
 * 
 * An Animator instance needs to be updated every frame, in order for it to keep
 * updating the animation pose of the associated entity. The currently playing
 * animation can be changed at any time using the doAnimation() method. The
 * Animator will keep looping the current animation until a new animation is
 * chosen.
 * 
 * The Animator calculates the desired current animation pose by interpolating
 * between the previous and next keyframes of the animation (based on the
 * current animation time). The Animator then updates the transforms all of the
 * joints each frame to match the current desired animation pose.
 * 
 * @author Karl
 *
 */
public class Animator {

	private final AnimatedModel entity;

	
	public Animator(AnimatedModel entity) {
		this.entity = entity;
		
	}

	public void doAnimation(Vector3f pose) {
		AnimationApp.getPlanner().initializePlanner(1, RobotSpecs.getPreviousEndPose(),
				RobotSpecs.getPreviousBasePose(), pose, new Vector3f(-90, 0, 0));
	}

	public void update() {
		if(AnimationApp.getPlanner().isFinished())
				return;
		List<Vector3f> poseInfo = AnimationApp.getPlanner().executePlanner();
//		System.out.println("Planned: "+poseInfo);
		RobotSpecs.setPreviousBasePose(poseInfo.get(0));
		applyPoseToJoints(entity.getRootJoint(), new Matrix4f().setTranslation(poseInfo.get(0)), poseInfo.get(1));
	}

	public void applyPoseToJoints(Joint joint, Matrix4f parentTransform, Vector3f thetas) {
		
		Vector3f translation = new Vector3f();
		joint.getLocalBindTransform().getTranslation(translation);
		float theta = 0;
		Matrix4f currentLocalTransform = new Matrix4f();
		if(joint.index == 0) {
			theta = thetas.x;
			currentLocalTransform.rotate((float)Math.toRadians(theta), new Vector3f(0, 1, 0));
		} else if(joint.index == 1) {
			theta = thetas.y;
			currentLocalTransform.rotate((float)Math.toRadians(theta), new Vector3f(0, 0, 1));
		}  else if(joint.index == 2) {
			theta = thetas.z;
			currentLocalTransform.rotate((float)Math.toRadians(theta), new Vector3f(0, 0, 1));
		} else
			theta = 0;
		currentLocalTransform.translate(new Vector3f(translation.x + ((float) (translation.y/2 * Math.sin(Math.toRadians(theta)))), (translation.y / 2) + (float) (translation.y/2 * Math.cos(Math.toRadians(theta))), translation.z));
		
		Matrix4f currentTransform = new Matrix4f().mul(parentTransform).mul(currentLocalTransform);
		for (Joint childJoint : joint.children) {
			applyPoseToJoints(childJoint, currentTransform, thetas);
		}
		currentTransform.mul(joint.getInverseBindTransform(), currentTransform);
		joint.setAnimationTransform(currentTransform);
	}
	
}
