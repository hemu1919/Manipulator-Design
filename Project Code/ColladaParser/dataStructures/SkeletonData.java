package dataStructures;

import org.joml.Matrix4f;
import org.joml.Vector3f;

public class SkeletonData {
	
	public final int jointCount;
	public final JointData headJoint;
	
	public SkeletonData(int jointCount, JointData headJoint){
		this.jointCount = jointCount;
		this.headJoint = headJoint;
		processJoint(headJoint);
	}

	public void processJoint(JointData joint) {
		if(joint.index == 2) {
			joint.bindLocalTransform.m31(joint.bindLocalTransform.m31() - 1.25f / 2);
		}
		for(JointData data : joint.children)
			processJoint(data);
	}
	
}
