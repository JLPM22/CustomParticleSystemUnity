using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public abstract class Obstacle : MonoBehaviour
    {
        [Range(0.0f, 1.0f)] public float Friction = 0.0f;

        public abstract bool HasCollisionParticle(Particle p);
        public abstract void CorrectCollisionParticle(Particle p, float deltaTime);
    }
}
