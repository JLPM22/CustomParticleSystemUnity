using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class Plane : Obstacle
    {
        [Range(0.0f, 1.0f)] public float Friction = 0.0f;

        private Vector3 Normal;
        private float D; // ax + by + cz + D = 0

        private void Awake()
        {
            Normal = transform.up;
            D = -Vector3.Dot(Normal, transform.position);
        }

        public override bool HasCollisionParticle(Particle p)
        {
            float sign = Vector3.Dot(p.Position, Normal) + D;
            sign *= Vector3.Dot(p.PreviousPosition, Normal) + D;
            return sign <= 0;
        }

        public override void CorrectCollisionParticle(Particle p)
        {
            p.Position = p.Position - (1 + p.Bouncing) * (Vector3.Dot(p.Position, Normal) + D) * Normal;

            // Friction
            Vector3 velocityNormal = Vector3.Dot(p.Velocity, Normal) * Normal;
            Vector3 velocityTangent = p.Velocity - velocityNormal;

            // Elastic collision
            p.Velocity = p.Velocity - (1 + p.Bouncing) * Vector3.Dot(p.Velocity, Normal) * Normal;

            // Apply friction
            p.Velocity = p.Velocity - Friction * velocityTangent;
        }
    }
}