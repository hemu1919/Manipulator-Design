package animatedModel;

//import org.lwjgl.util.vector.Matrix4f;
import org.joml.Matrix4f;
import org.joml.Vector3f;

import animation.Animator;
import openglObjects.Vao;
import textures.Texture;

public class AnimatedModel {

	private final Vao model;
	private final Texture texture;

	private final Joint rootJoint;
	private final int jointCount;

	private final Animator animator;
	
	public AnimatedModel(Vao model, Texture texture, Joint rootJoint, int jointCount) {
		this.model = model;
		this.texture = texture;
		this.rootJoint = rootJoint;
		this.jointCount = jointCount;
		this.animator = new Animator(this);
		rootJoint.calcInverseBindTransform(new Matrix4f());
	}

	public Vao getModel() {
		return model;
	}

	public Texture getTexture() {
		return texture;
	}

	public Joint getRootJoint() {
		return rootJoint;
	}

	public void delete() {
		model.delete();
		texture.delete();
	}

	public void doAnimation(Vector3f pose) {
		animator.doAnimation(pose);
	}

	public void renderObject(Matrix4f parentTransform) {
		animator.applyPoseToJoints(getRootJoint(), parentTransform, new Vector3f(0, 0, 0));
	}
	
	public void update() {
		animator.update();
	}

	public Matrix4f[] getJointTransforms() {
		Matrix4f[] jointMatrices = new Matrix4f[jointCount];
		addJointsToArray(rootJoint, jointMatrices);
		return jointMatrices;
	}

	private void addJointsToArray(Joint headJoint, Matrix4f[] jointMatrices) {
		jointMatrices[headJoint.index] = headJoint.getAnimatedTransform();
		for (Joint childJoint : headJoint.children) {
			addJointsToArray(childJoint, jointMatrices);
		}
	}

}
