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
import ander.render.math.FastMath;
import ander.render.math.Quaternion;
import ander.render.math.Vector3f;
import ander.render.math.Vector4f;
import ander.render.renderer.Caps;
import ander.render.renderer.RenderManager;
import ander.render.renderer.Renderer;
import general.nieuw.scene.Geometry;
import general.nieuw.scene.light.AmbientLight;
import general.nieuw.scene.light.DirectionalLight;
import general.nieuw.scene.light.Light;
import general.nieuw.scene.light.LightList;
import general.nieuw.scene.light.PointLight;
import general.nieuw.scene.light.SpotLight;
import general.nieuw.scene.material.RenderState;
import general.nieuw.scene.material.TechniqueDef;
import general.nieuw.scene.shader.DefineList;
import general.nieuw.scene.shader.Shader;
import general.nieuw.scene.shader.Uniform;
import general.nieuw.scene.shader.VarType;
import general.nieuw.util.TempVars;

import java.util.EnumSet;

public final class MultiPassLightingLogic extends DefaultTechniqueDefLogic {

    private static final RenderState ADDITIVE_LIGHT = new RenderState();
    private static final Quaternion NULL_DIR_LIGHT = new Quaternion(0, -1, 0, -1);
    
    private final ColorRGBA ambientLightColor = new ColorRGBA(0, 0, 0, 1);
    
    static {
        ADDITIVE_LIGHT.setBlendMode(RenderState.BlendMode.AlphaAdditive);
        ADDITIVE_LIGHT.setDepthWrite(false);
    }
    
    public MultiPassLightingLogic(TechniqueDef techniqueDef) {
        super(techniqueDef);
    }

    @Override
    public void render(RenderManager renderManager, Shader shader, Geometry geometry, LightList lights, int lastTexUnit) {
        Renderer r = renderManager.getRenderer();
        Uniform lightDir = shader.getUniform("g_LightDirection");
        Uniform lightColor = shader.getUniform("g_LightColor");
        Uniform lightPos = shader.getUniform("g_LightPosition");
        Uniform ambientColor = shader.getUniform("g_AmbientLightColor");
        boolean isFirstLight = true;
        boolean isSecondLight = false;
        
        getAmbientColor(lights, false, ambientLightColor);

        for (int i = 0; i < lights.size(); i++) {
            Light l = lights.get(i);
            if (l instanceof AmbientLight) {
                continue;
            }

            if (isFirstLight) {
                // set ambient color for first light only
                ambientColor.setValue(VarType.Vector4, ambientLightColor);
                isFirstLight = false;
                isSecondLight = true;
            } else if (isSecondLight) {
                ambientColor.setValue(VarType.Vector4, ColorRGBA.Black);
                // apply additive blending for 2nd and future lights
                r.applyRenderState(ADDITIVE_LIGHT);
                isSecondLight = false;
            }

            TempVars vars = TempVars.get();
            Quaternion tmpLightDirection = vars.quat1;
            Quaternion tmpLightPosition = vars.quat2;
            ColorRGBA tmpLightColor = vars.color;
            Vector4f tmpVec = vars.vect4f1;

            ColorRGBA color = l.getColor();
            tmpLightColor.set(color);
            tmpLightColor.a = l.getType().getId();
            lightColor.setValue(VarType.Vector4, tmpLightColor);

            switch (l.getType()) {
                case Directional:
                    DirectionalLight dl = (DirectionalLight) l;
                    Vector3f dir = dl.getDirection();
                    //FIXME : there is an inconstency here due to backward
                    //compatibility of the lighting shader.
                    //The directional light direction is passed in the
                    //LightPosition uniform. The lighting shader needs to be
                    //reworked though in order to fix this.
                    tmpLightPosition.set(dir.getX(), dir.getY(), dir.getZ(), -1);
                    lightPos.setValue(VarType.Vector4, tmpLightPosition);
                    tmpLightDirection.set(0, 0, 0, 0);
                    lightDir.setValue(VarType.Vector4, tmpLightDirection);
                    break;
                case Point:
                    PointLight pl = (PointLight) l;
                    Vector3f pos = pl.getPosition();
                    float invRadius = pl.getInvRadius();

                    tmpLightPosition.set(pos.getX(), pos.getY(), pos.getZ(), invRadius);
                    lightPos.setValue(VarType.Vector4, tmpLightPosition);
                    tmpLightDirection.set(0, 0, 0, 0);
                    lightDir.setValue(VarType.Vector4, tmpLightDirection);
                    break;
                case Spot:
                    SpotLight sl = (SpotLight) l;
                    Vector3f pos2 = sl.getPosition();
                    Vector3f dir2 = sl.getDirection();
                    float invRange = sl.getInvSpotRange();
                    float spotAngleCos = sl.getPackedAngleCos();

                    tmpLightPosition.set(pos2.getX(), pos2.getY(), pos2.getZ(), invRange);
                    lightPos.setValue(VarType.Vector4, tmpLightPosition);

                    //We transform the spot direction in view space here to save 5 varying later in the lighting shader
                    //one vec4 less and a vec4 that becomes a vec3
                    //the downside is that spotAngleCos decoding happens now in the frag shader.
                    tmpVec.set(dir2.getX(), dir2.getY(), dir2.getZ(), 0);
                    renderManager.getCurrentCamera().getViewMatrix().mult(tmpVec, tmpVec);
                    tmpLightDirection.set(tmpVec.getX(), tmpVec.getY(), tmpVec.getZ(), spotAngleCos);

                    lightDir.setValue(VarType.Vector4, tmpLightDirection);

                    break;
                case Probe:
                    break;
                default:
                    throw new UnsupportedOperationException("Unknown type of light: " + l.getType());
            }
            vars.release();
            r.setShader(shader);
            renderMeshFromGeometry(r, geometry);
        }

        if (isFirstLight) {
            // Either there are no lights at all, or only ambient lights.
            // Render a dummy "normal light" so we can see the ambient color.
            ambientColor.setValue(VarType.Vector4, getAmbientColor(lights, false, ambientLightColor));
            lightColor.setValue(VarType.Vector4, ColorRGBA.BlackNoAlpha);
            lightPos.setValue(VarType.Vector4, NULL_DIR_LIGHT);
            r.setShader(shader);
            renderMeshFromGeometry(r, geometry);
        }
    }
}
