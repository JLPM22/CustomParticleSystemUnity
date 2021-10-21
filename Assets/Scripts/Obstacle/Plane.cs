using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class Plane : Obstacle
    {
        public Vector3 Normal { get; private set; }
        public float D { get; private set; } // ax + by + cz + D = 0

        private void Update()
        {
            Normal = transform.up;
            D = -Vector3.Dot(Normal, transform.position);
        }

        public override bool HasCollisionParticle(Particle p)
        {
            return IsCrossingPlane(p, Normal, D);
        }

        public override void CorrectCollisionParticle(Particle p, float deltaTime)
        {
            CollisionPlaneParticle(p, Normal, D, Friction, deltaTime);
        }

        public static void CollisionPlaneParticle(Particle p, Vector3 N, float d, float friction, float deltaTime)
        {
            float dotVN = Vector3.Dot(p.Velocity, N);

            // Friction
            Vector3 velocityNormal = dotVN * N;
            Vector3 velocityTangent = p.Velocity - velocityNormal;

            // Elastic collision
            p.SetVelocity(p.Velocity - (1 + Particle.Bouncing) * velocityNormal);

            // Apply friction
            p.SetVelocity(p.Velocity - friction * velocityTangent);

            // Update position
            p.SetPosition(p.Position - (1 + Particle.Bouncing) * (Vector3.Dot(p.GetBoundary(p.Position, -N), N) + d) * N);
        }

        public static bool IsCrossingPlane(Particle p, Vector3 N, float d)
        {
            float sign = Vector3.Dot(p.GetBoundary(p.Position, -N), N) + d;
            sign *= Vector3.Dot(p.GetBoundary(p.PreviousPosition, N), N) + d;
            return sign <= 0;
        }

        public static bool IsNegativeSpacePlane(Particle p, Vector3 N, float d)
        {
            float sign = Vector3.Dot(p.GetBoundary(p.Position, -N), N) + d;
            return sign <= 0;
        }

        public override int GetPriority()
        {
            return 100;
        }
    }
}