/*
 * Copyright (c) 2009-2015 jMonkeyEngine
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
package general.nieuw.scene.material.logic;

import com.jme3.asset.AssetManager;

import ander.render.math.ColorRGBA;
import ander.render.math.Matrix4f;
import ander.render.math.Vector3f;
import ander.render.renderer.Caps;
import ander.render.renderer.RenderManager;
import ander.render.renderer.Renderer;
import general.nieuw.scene.Geometry;
import general.nieuw.scene.light.DirectionalLight;
import general.nieuw.scene.light.Light;
import general.nieuw.scene.light.LightList;
import general.nieuw.scene.light.PointLight;
import general.nieuw.scene.light.SpotLight;
import general.nieuw.scene.material.TechniqueDef;
import general.nieuw.scene.shader.DefineList;
import general.nieuw.scene.shader.Shader;
import general.nieuw.scene.shader.Uniform;
import general.nieuw.scene.shader.VarType;

import java.util.ArrayList;
import java.util.EnumSet;

/**
 * Rendering logic for static pass.
 *
 * @author Kirill Vainer
 */
public final class StaticPassLightingLogic extends DefaultTechniqueDefLogic {

    private static final String DEFINE_NUM_DIR_LIGHTS = "NUM_DIR_LIGHTS";
    private static final String DEFINE_NUM_POINT_LIGHTS = "NUM_POINT_LIGHTS";
    private static final String DEFINE_NUM_SPOT_LIGHTS = "NUM_SPOT_LIGHTS";

    private final int numDirLightsDefineId;
    private final int numPointLightsDefineId;
    private final int numSpotLightsDefineId;

    private final ArrayList<DirectionalLight> tempDirLights = new ArrayList<DirectionalLight>();
    private final ArrayList<PointLight> tempPointLights = new ArrayList<PointLight>();
    private final ArrayList<SpotLight> tempSpotLights = new ArrayList<SpotLight>();

    private final ColorRGBA ambientLightColor = new ColorRGBA(0, 0, 0, 1);
    private final Vector3f tempPosition = new Vector3f();
    private final Vector3f tempDirection = new Vector3f();

    public StaticPassLightingLogic(TechniqueDef techniqueDef) {
        super(techniqueDef);

        numDirLightsDefineId = techniqueDef.addShaderUnmappedDefine(DEFINE_NUM_DIR_LIGHTS, VarType.Int);
        numPointLightsDefineId = techniqueDef.addShaderUnmappedDefine(DEFINE_NUM_POINT_LIGHTS, VarType.Int);
        numSpotLightsDefineId = techniqueDef.addShaderUnmappedDefine(DEFINE_NUM_SPOT_LIGHTS, VarType.Int);
    }

    @Override
    public Shader makeCurrent(AssetManager assetManager, RenderManager renderManager,
            EnumSet<Caps> rendererCaps, LightList lights, DefineList defines) {

        // TODO: if it ever changes that render isn't called
        // right away with the same geometry after makeCurrent, it would be
        // a problem.
        // Do a radix sort.
        tempDirLights.clear();
        tempPointLights.clear();
        tempSpotLights.clear();
        for (Light light : lights) {
            switch (light.getType()) {
                case Directional:
                    tempDirLights.add((DirectionalLight) light);
                    break;
                case Point:
                    tempPointLights.add((PointLight) light);
                    break;
                case Spot:
                    tempSpotLights.add((SpotLight) light);
                    break;
            }
        }

        defines.set(numDirLightsDefineId, tempDirLights.size());
        defines.set(numPointLightsDefineId, tempPointLights.size());
        defines.set(numSpotLightsDefineId, tempSpotLights.size());

        return techniqueDef.getShader(assetManager, rendererCaps, defines);
    }

    private void transformDirection(Matrix4f viewMatrix, Vector3f direction) {
        viewMatrix.multNormal(direction, direction);
    }

    private void transformPosition(Matrix4f viewMatrix, Vector3f location) {
        viewMatrix.mult(location, location);
    }

    private void updateLightListUniforms(Matrix4f viewMatrix, Shader shader, LightList lights) {
        Uniform ambientColor = shader.getUniform("g_AmbientLightColor");
        ambientColor.setValue(VarType.Vector4, getAmbientColor(lights, true, ambientLightColor));

        Uniform lightData = shader.getUniform("g_LightData");

        int totalSize = tempDirLights.size() * 2
                + tempPointLights.size() * 2
                + tempSpotLights.size() * 3;
        lightData.setVector4Length(totalSize);

        int index = 0;
        for (DirectionalLight light : tempDirLights) {
            ColorRGBA color = light.getColor();
            tempDirection.set(light.getDirection());
            transformDirection(viewMatrix, tempDirection);
            lightData.setVector4InArray(color.r, color.g, color.b, 1f, index++);
            lightData.setVector4InArray(tempDirection.x, tempDirection.y, tempDirection.z, 1f, index++);
        }

        for (PointLight light : tempPointLights) {
            ColorRGBA color = light.getColor();
            tempPosition.set(light.getPosition());
            float invRadius = light.getInvRadius();
            transformPosition(viewMatrix, tempPosition);
            lightData.setVector4InArray(color.r, color.g, color.b, 1f, index++);
            lightData.setVector4InArray(tempPosition.x, tempPosition.y, tempPosition.z, invRadius, index++);
        }

        for (SpotLight light : tempSpotLights) {
            ColorRGBA color = light.getColor();
            Vector3f pos = light.getPosition();
            Vector3f dir = light.getDirection();

            tempPosition.set(light.getPosition());
            tempDirection.set(light.getDirection());
            transformPosition(viewMatrix, tempPosition);
            transformDirection(viewMatrix, tempDirection);

            float invRange = light.getInvSpotRange();
            float spotAngleCos = light.getPackedAngleCos();
            lightData.setVector4InArray(color.r, color.g, color.b, 1f, index++);
            lightData.setVector4InArray(tempPosition.x, tempPosition.y, tempPosition.z, invRange, index++);
            lightData.setVector4InArray(tempDirection.x, tempDirection.y, tempDirection.z, spotAngleCos, index++);
        }
    }

    @Override
    public void render(RenderManager renderManager, Shader shader, Geometry geometry, LightList lights, int lastTexUnit) {
        Renderer renderer = renderManager.getRenderer();
        Matrix4f viewMatrix = renderManager.getCurrentCamera().getViewMatrix();
        updateLightListUniforms(viewMatrix, shader, lights);
        renderer.setShader(shader);
        renderMeshFromGeometry(renderer, geometry);
    }

}
