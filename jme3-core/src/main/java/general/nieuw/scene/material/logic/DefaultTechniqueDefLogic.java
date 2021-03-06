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
import ander.render.renderer.Caps;
import ander.render.renderer.RenderManager;
import ander.render.renderer.Renderer;
import general.nieuw.scene.Geometry;
import general.nieuw.scene.Mesh;
import general.nieuw.scene.instancing.InstancedGeometry;
import general.nieuw.scene.light.*;
import general.nieuw.scene.material.TechniqueDef;
import general.nieuw.scene.shader.DefineList;
import general.nieuw.scene.shader.Shader;

import java.util.EnumSet;

public class DefaultTechniqueDefLogic implements TechniqueDefLogic {

    protected final TechniqueDef techniqueDef;

    public DefaultTechniqueDefLogic(TechniqueDef techniqueDef) {
        this.techniqueDef = techniqueDef;
    }

    @Override
    public Shader makeCurrent(AssetManager assetManager, RenderManager renderManager,
            EnumSet<Caps> rendererCaps, LightList lights, DefineList defines) {
        return techniqueDef.getShader(assetManager, rendererCaps, defines);
    }

    public static void renderMeshFromGeometry(Renderer renderer, Geometry geom) {
        Mesh mesh = geom.getMesh();
        int lodLevel = geom.getLodLevel();
        if (geom instanceof InstancedGeometry) {
            InstancedGeometry instGeom = (InstancedGeometry) geom;
            renderer.renderMesh(mesh, lodLevel, instGeom.getActualNumInstances(),
                    instGeom.getAllInstanceData());
        } else {
            renderer.renderMesh(mesh, lodLevel, 1, null);
        }
    }

    protected static ColorRGBA getAmbientColor(LightList lightList, boolean removeLights, ColorRGBA ambientLightColor) {
        ambientLightColor.set(0, 0, 0, 1);
        for (int j = 0; j < lightList.size(); j++) {
            Light l = lightList.get(j);
            if (l instanceof AmbientLight) {
                ambientLightColor.addLocal(l.getColor());
                if (removeLights) {
                    lightList.remove(l);
                }
            }
        }
        ambientLightColor.a = 1.0f;
        return ambientLightColor;
    }



    @Override
    public void render(RenderManager renderManager, Shader shader, Geometry geometry, LightList lights, int lastTexUnit) {
        Renderer renderer = renderManager.getRenderer();
        renderer.setShader(shader);
        renderMeshFromGeometry(renderer, geometry);
    }
}
