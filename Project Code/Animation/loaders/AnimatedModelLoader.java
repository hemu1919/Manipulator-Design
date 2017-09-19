package loaders;

import org.joml.Matrix4f;
import org.joml.Vector3f;

import animatedModel.AnimatedModel;
import animatedModel.Joint;
import colladaLoader.ColladaLoader;
import dataStructures.AnimatedModelData;
import dataStructures.JointData;
import dataStructures.MeshData;
import dataStructures.SkeletonData;
import main.GeneralSettings;
import main.RobotSpecs;
import openglObjects.Vao;
import textures.Texture;
import utils.MyFile;

public class AnimatedModelLoader {

	private static float la, lb, lc, ld, le;
	
	/**
	 * Creates an AnimatedEntity from the data in an entity file. It loads up
	 * the collada model data, stores the extracted data in a VAO, sets up the
	 * joint heirarchy, and loads up the entity's texture.
	 * 
	 * @param entityFile
	 *            - the file containing the data for the entity.
	 * @return The animated entity (no animation applied though)
	 */
	public static AnimatedModel loadEntity(MyFile modelFile, MyFile textureFile, Vector3f scale) {
		AnimatedModelData entityData = ColladaLoader.loadColladaModel(modelFile, GeneralSettings.MAX_WEIGHTS, scale);
		Vao model = createVao(entityData.getMeshData());
		Texture texture = loadTexture(textureFile);
		SkeletonData skeletonData = entityData.getJointsData();
		Joint headJoint = createJoints(skeletonData.headJoint);
		RobotSpecs.setL2((lb + lc) / 2);
		RobotSpecs.setL3((lb / 2) + ld);
		return new AnimatedModel(model, texture, headJoint, skeletonData.jointCount);
	}

	/**
	 * Loads up the diffuse texture for the model.
	 * 
	 * @param textureFile
	 *            - the texture file.
	 * @return The diffuse texture.
	 */
	private static Texture loadTexture(MyFile textureFile) {
		Texture diffuseTexture = Texture.newTexture(textureFile).anisotropic().create();
		return diffuseTexture;
	}

	/**
	 * Constructs the joint-hierarchy skeleton from the data extracted from the
	 * collada file.
	 * 
	 * @param data
	 *            - the joints data from the collada file for the head joint.
	 * @return The created joint, with all its descendants added.
	 */
	private static Joint createJoints(JointData data) {
		Joint joint = new Joint(data.index, data.nameId, data.bindLocalTransform);
//		Matrix4f matrix = data.bindLocalTransform;
//		Vector3f rotation = new Vector3f();
//		matrix.getEulerAnglesZYX(rotation);
//		Vector3f translation = new Vector3f();
//		matrix.getTranslation(translation);
//		System.out.println(data.nameId);
//		System.out.println(matrix);
//		System.out.println(rotation);
//		double beta = Math.toDegrees(Math.atan2(-matrix.m13(), Math.sqrt((matrix.m11() * matrix.m11()) + (matrix.m12() * matrix.m12()))));
//		System.out.println("\nbeta: "+beta);
//		double alpha = Math.toDegrees(Math.atan2(matrix.m12() / Math.cos(Math.toRadians(beta)), matrix.m11() / Math.cos(Math.toRadians(beta))));
//		System.out.println("alpha: "+alpha);
//		double gamma = Math.toDegrees(Math.atan2(matrix.m23() / Math.cos(Math.toRadians(beta)), matrix.m33() / Math.cos(Math.toRadians(beta))));
//		System.out.println("gamma: "+gamma);
//		System.out.println("\n"+translation);
		Vector3f translation = new Vector3f();
		data.bindLocalTransform.getTranslation(translation);
		RobotSpecs.setJointPose(data.index, translation);
		if(data.index == 0)
			la = data.bindLocalTransform.m31();
		else if(data.index == 1)
			lb = data.bindLocalTransform.m31();
		else if(data.index == 2)
			lc = data.bindLocalTransform.m31();
		else if(data.index == 3)
			ld = data.bindLocalTransform.m31();
		else if(data.index == 4)
			le = data.bindLocalTransform.m31();
		for (JointData child : data.children) {
			joint.addChild(createJoints(child));
		}
		return joint;
	}

	/**
	 * Stores the mesh data in a VAO.
	 * 
	 * @param data
	 *            - all the data about the mesh that needs to be stored in the
	 *            VAO.
	 * @return The VAO containing all the mesh data for the model.
	 */
	private static Vao createVao(MeshData data) {
		Vao vao = Vao.create();
		vao.bind();
		vao.createIndexBuffer(data.getIndices());
		vao.createAttribute(0, data.getVertices(), 3);
		vao.createAttribute(1, data.getTextureCoords(), 2);
		vao.createAttribute(2, data.getNormals(), 3);
		vao.createIntAttribute(3, data.getJointIds(), 3);
		vao.createAttribute(4, data.getVertexWeights(), 3);
		vao.unbind();
		return vao;
	}

}
