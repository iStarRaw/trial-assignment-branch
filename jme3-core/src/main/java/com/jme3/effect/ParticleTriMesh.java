/*
 * Copyright (c) 2009-2012 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.effect;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Format;
import com.jme3.scene.VertexBuffer.Usage;
import com.jme3.util.BufferUtils;
import com.jme3.util.TempVars;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

public class ParticleTriMesh extends ParticleMesh {

	private int imagesX = 1;
	private int imagesY = 1;
	private boolean uniqueTexCoords = false;
	// private ParticleComparator comparator = new ParticleComparator();
	private ParticleEmitter emitter;
	// private Particle[] particlesCopy;

	@Override
	public void initParticleData(ParticleEmitter emitter, int numParticles) {
		setMode(Mode.Triangles);

		this.emitter = emitter;

		// particlesCopy = new Particle[numParticles];

		// set positions
		FloatBuffer pb = BufferUtils.createVector3Buffer(numParticles * 4);
		// if the buffer is already set only update the data
		VertexBuffer buf = getBuffer(VertexBuffer.Type.Position);
		if (buf != null) {
			buf.updateData(pb);
		} else {
			VertexBuffer pvb = new VertexBuffer(VertexBuffer.Type.Position);
			pvb.setupData(Usage.Stream, 3, Format.Float, pb);
			setBuffer(pvb);
		}

		// set colors
		ByteBuffer cb = BufferUtils.createByteBuffer(numParticles * 4 * 4);
		buf = getBuffer(VertexBuffer.Type.Color);
		if (buf != null) {
			buf.updateData(cb);
		} else {
			VertexBuffer cvb = new VertexBuffer(VertexBuffer.Type.Color);
			cvb.setupData(Usage.Stream, 4, Format.UnsignedByte, cb);
			cvb.setNormalized(true);
			setBuffer(cvb);
		}

		// set texcoords
		FloatBuffer tb = BufferUtils.createVector2Buffer(numParticles * 4);
		uniqueTexCoords = false;
		for (int i = 0; i < numParticles; i++) {
			tb.put(0f).put(1f);
			tb.put(1f).put(1f);
			tb.put(0f).put(0f);
			tb.put(1f).put(0f);
		}
		tb.flip();

		buf = getBuffer(VertexBuffer.Type.TexCoord);
		if (buf != null) {
			buf.updateData(tb);
		} else {
			VertexBuffer tvb = new VertexBuffer(VertexBuffer.Type.TexCoord);
			tvb.setupData(Usage.Static, 2, Format.Float, tb);
			setBuffer(tvb);
		}

		// set indices
		ShortBuffer ib = BufferUtils.createShortBuffer(numParticles * 6);
		for (int i = 0; i < numParticles; i++) {
			int startIdx = (i * 4);

			// triangle 1
			ib.put((short) (startIdx + 1)).put((short) (startIdx + 0)).put((short) (startIdx + 2));

			// triangle 2
			ib.put((short) (startIdx + 1)).put((short) (startIdx + 2)).put((short) (startIdx + 3));
		}
		ib.flip();

		buf = getBuffer(VertexBuffer.Type.Index);
		if (buf != null) {
			buf.updateData(ib);
		} else {
			VertexBuffer ivb = new VertexBuffer(VertexBuffer.Type.Index);
			ivb.setupData(Usage.Static, 3, Format.UnsignedShort, ib);
			setBuffer(ivb);
		}

		updateCounts();
	}

	@Override
	public void setImagesXY(int imagesX, int imagesY) {
		this.imagesX = imagesX;
		this.imagesY = imagesY;
		if (imagesX != 1 || imagesY != 1) {
			uniqueTexCoords = true;
			getBuffer(VertexBuffer.Type.TexCoord).setUsage(Usage.Stream);
		}
	}
	
	class UpdateParticleDataHelper {

		private VertexBuffer pvb;
		private FloatBuffer positions;

		private VertexBuffer cvb;
		private ByteBuffer colors;

		private VertexBuffer tvb;
		private FloatBuffer texcoords;
		
		private Vector3f camUp;
		private Vector3f camLeft; 
		private Vector3f camDir;
		
		private Vector3f up, left;
		private Vector3f faceNormal;
		
		private Camera cam;
		
		boolean facingVelocity;
		
		UpdateParticleDataHelper(Camera cam) {
			this.cam = cam;
			setPosistions();
			setColors();
			setTextcoords();
			setCamAxis();
			up = new Vector3f();
			left = new Vector3f();
			faceNormal = emitter.getFaceNormal();
			facingVelocity = emitter.isFacingVelocity();
			
		}

		private void setCamAxis() {
			camUp = this.cam.getUp();
			camLeft = this.cam.getLeft();
			camDir = this.cam.getDirection();
		}

		private void setTextcoords() {
			tvb = getBuffer(VertexBuffer.Type.TexCoord);
			texcoords = (FloatBuffer) tvb.getData();
		}

		private void setColors() {
			cvb = getBuffer(VertexBuffer.Type.Color);
			colors = (ByteBuffer) cvb.getData();
		}

		private void setPosistions() {
			pvb = getBuffer(VertexBuffer.Type.Position);
			positions = (FloatBuffer) pvb.getData();
		}
			
	}

	@Override
	public void updateParticleData(Particle[] particles, Camera cam, Matrix3f inverseRotation) {
		UpdateParticleDataHelper updh = new UpdateParticleDataHelper(cam);
		multMatrixByVector(inverseRotation, updh);
		if (!updh.facingVelocity) {
			updateVectorCamAxis(updh);
		}
		clearVertexBuffers(updh);
		updateParticles(particles, updh);
		clearVertexBuffers(updh);
		forceRenderDataToGPU(updh);
	}

	private void updateParticles(Particle[] particles, UpdateParticleDataHelper updh) {
		for (int i = 0; i < particles.length; i++) {
			Particle p = particles[i];
			boolean dead = p.life == 0;
			if (dead) {
				updateToDead(updh);
				continue;
			}
			updateVectors(updh, p);
			updateVertexBuffers(updh, p);
		}
	}

	private void updateVertexBuffers(UpdateParticleDataHelper updh, Particle p) {
		updateVertexBufferPositions(updh, p);

		if (uniqueTexCoords) {
			updateVertexBufferTexCoords(updh, p);
		}
		updateVertexBufferColors(updh, p);
	}

	private void forceRenderDataToGPU(UpdateParticleDataHelper updh) {
		if (uniqueTexCoords) {
			updh.tvb.updateData(updh.texcoords);
		}
		updh.pvb.updateData(updh.positions);
		updh.cvb.updateData(updh.colors);
	}

	private void clearVertexBuffers(UpdateParticleDataHelper updh) {
		updh.positions.clear();
		updh.colors.clear();
		updh.texcoords.clear();
	}

	private void updateVertexBufferTexCoords(UpdateParticleDataHelper updh, Particle p) {
		int imgX = p.imageIndex % imagesX;
		int imgY = (p.imageIndex - imgX) / imagesY;

		float startX = ((float) imgX) / imagesX;
		float startY = ((float) imgY) / imagesY;
		float endX = startX + (1f / imagesX);
		float endY = startY + (1f / imagesY);

		updh.texcoords.put(startX).put(endY);
		updh.texcoords.put(endX).put(endY);
		updh.texcoords.put(startX).put(startY);
		updh.texcoords.put(endX).put(startY);
	}

	private void updateVertexBufferColors(UpdateParticleDataHelper updh, Particle p) {
		int abgr = p.color.asIntABGR();
		updh.colors.putInt(abgr);
		updh.colors.putInt(abgr);
		updh.colors.putInt(abgr);
		updh.colors.putInt(abgr);
	}

	

	private void updateVertexBufferPositions(UpdateParticleDataHelper updh, Particle p) {
		updh.positions.put(p.position.x + updh.left.x + updh.up.x).put(p.position.y + updh.left.y + updh.up.y)
				.put(p.position.z + updh.left.z + updh.up.z);

		updh.positions.put(p.position.x - updh.left.x + updh.up.x).put(p.position.y - updh.left.y + updh.up.y)
				.put(p.position.z - updh.left.z + updh.up.z);

		updh.positions.put(p.position.x + updh.left.x - updh.up.x).put(p.position.y + updh.left.y - updh.up.y)
				.put(p.position.z + updh.left.z - updh.up.z);

		updh.positions.put(p.position.x - updh.left.x - updh.up.x).put(p.position.y - updh.left.y - updh.up.y)
				.put(p.position.z - updh.left.z - updh.up.z);
	}

	private void updateVectors(UpdateParticleDataHelper updh, Particle p) {
		if (updh.facingVelocity) {
			updateFacingVelocity(updh, p);
		} else if (updh.faceNormal != null) {
			updateFaceNormal(updh, p);
		} else if (p.angle != 0) {
			updateVectorAngle(updh, p);
		} else {
			updateVectorCamAxis(updh);
			multVectorByScalor(updh, p);
		}
	}

	private void updateVectorAngle(UpdateParticleDataHelper updh, Particle p) {
		float cos = FastMath.cos(p.angle) * p.size;
		float sin = FastMath.sin(p.angle) * p.size;

		updh.left.x = updh.camLeft.x * cos + updh.camUp.x * sin;
		updh.left.y = updh.camLeft.y * cos + updh.camUp.y * sin;
		updh.left.z = updh.camLeft.z * cos + updh.camUp.z * sin;

		updh.up.x = updh.camLeft.x * -sin + updh.camUp.x * cos;
		updh.up.y = updh.camLeft.y * -sin + updh.camUp.y * cos;
		updh.up.z = updh.camLeft.z * -sin + updh.camUp.z * cos;
	}
	
	private void updateVectorCamAxis(UpdateParticleDataHelper updh) {
		updh.up.set(updh.camUp);
		updh.left.set(updh.camLeft);
	}

	private void updateFaceNormal(UpdateParticleDataHelper updh, Particle p) {
		updh.up.set(updh.faceNormal).crossLocal(Vector3f.UNIT_X);
		updh.faceNormal.cross(updh.up, updh.left);
		multVectorByScalor(updh, p);
		if (p.angle != 0) {
			TempVars vars = TempVars.get();
			vars.vect1.set(updh.faceNormal).normalizeLocal();
			vars.quat1.fromAngleNormalAxis(p.angle, vars.vect1);
			vars.quat1.multLocal(updh.left);
			vars.quat1.multLocal(updh.up);
			vars.release();
		}
	}

	private void updateFacingVelocity(UpdateParticleDataHelper updh, Particle p) {
		updh.left.set(p.velocity).normalizeLocal();
		updh.camDir.cross(updh.left, updh.up);
		multVectorByScalor(updh, p);
	}

	private void multVectorByScalor(UpdateParticleDataHelper updh, Particle p) {
		updh.up.multLocal(p.size);
		updh.left.multLocal(p.size);
	}

	private void updateToDead(UpdateParticleDataHelper updh) {
		updh.positions.put(0).put(0).put(0);
		updh.positions.put(0).put(0).put(0);
		updh.positions.put(0).put(0).put(0);
		updh.positions.put(0).put(0).put(0);	
	}

	private void multMatrixByVector(Matrix3f inverseRotation, UpdateParticleDataHelper updh) {
		inverseRotation.multLocal(updh.camUp);
		inverseRotation.multLocal(updh.camLeft);
		inverseRotation.multLocal(updh.camDir);
	}

}
