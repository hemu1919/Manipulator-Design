package main;

import org.joml.Matrix4f;
import org.joml.Vector3f;

import animatedModel.AnimatedModel;
import loaders.AnimatedModelLoader;
import utils.MyFile;


public class Entity {
	
	private Vector3f position;
	
	private AnimatedModel model;
	
	public Entity(Vector3f position, MyFile resFolder) {
		this.position = position;
		this.model = AnimatedModelLoader.loadEntity(new MyFile(resFolder, GeneralSettings.OBJECT_FILE),
				new MyFile(resFolder, GeneralSettings.DIFFUSE_FILE), new Vector3f(2, 2, 2));
		this.render();
	}

	public Vector3f getPosition() {
		return position;
	}
	
	private void render() {
		model.renderObject(new Matrix4f().setTranslation(position));
	}
	
	public AnimatedModel getModel() {
		return model;
	}
	
}
