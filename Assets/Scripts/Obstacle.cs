using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public abstract class Obstacle : MonoBehaviour
    {
        public abstract bool HasCollisionParticle(Particle p);
        public abstract void CorrectCollisionParticle(Particle p);
    }
}
