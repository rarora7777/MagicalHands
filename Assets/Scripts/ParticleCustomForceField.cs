//////////////////////////////////////////////////////////////////////////////////////
/// ParticleCustomForceField can be utilized to define a custom function for applying
/// forces onto particles emitted from a system. This is currently unused but could be
/// used in some kind of Advect() function. See ParticleEmitter.AdvectParticleDirect()
/// for inspiration.
//////////////////////////////////////////////////////////////////////////////////////

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GestureAnim
{
    public class ParticleCustomForceField
    {
        public enum FieldEffectType { Gravity, Wind};
        public bool Enabled { get; set; }
        public FieldEffectType EffectType { get; private set; }
        private readonly Func<Vector3, Vector3> force;
        private readonly Func<Vector3, Vector3> velocity;

        private int customDataIdx;
        private float dt;

        public ParticleCustomForceField(FieldEffectType type, Func<Vector3, Vector3> f, Func<Vector3, Vector3> v0, int cdataIdx)
        {
            EffectType = type;
            force = f;
            velocity = v0;
            customDataIdx = cdataIdx;
        }

        public Vector3 ApplyField(Vector3 position, ref ParticleCustomData cdata, float influence, int particleIdx)
        {
            var acc = force(position);
            var xn_1 = cdata.Xn_1[customDataIdx];
            var xn = cdata.Xn[customDataIdx];

            var dt = Time.deltaTime;
            var dt2 = dt * dt;

            Vector3 newPos;

            if (cdata.NumTimestep == 0)
            {
                newPos = xn + (velocity(xn) * dt) + (influence * 0.5f * dt2 * force(xn));
            }
            else
            {
                newPos = (2 * xn) - xn_1 + (influence * dt2 * force(xn));
            }

            cdata.Xn[customDataIdx] = newPos;
            cdata.Xn_1[customDataIdx] = xn;

            return newPos;
        }
    }
}