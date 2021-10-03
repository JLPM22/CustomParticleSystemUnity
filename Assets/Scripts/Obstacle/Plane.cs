using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class Plane : Obstacle
    {
        private Vector3 Normal
        {
            get { return transform.up; }
        }
        // ax + by + cz + D = 0
        private float D
        {
            get { return -Vector3.Dot(Normal, transform.position); }
        }

        public override bool HasCollisionParticle(Particle p)
        {
            return IsCrossingPlane(p, Normal, D);
        }

        public override void CorrectCollisionParticle(Particle p, float deltaTime)
        {
            CollisionPlaneParticle(p, Normal, D, Friction);
        }

        public static void CollisionPlaneParticle(Particle p, Vector3 N, float d, float friction)
        {
            p.Position = p.Position - (1 + p.Bouncing) * (Vector3.Dot(p.Position, N) + d) * N;

            // Friction
            Vector3 velocityNormal = Vector3.Dot(p.Velocity, N) * N;
            Vector3 velocityTangent = p.Velocity - velocityNormal;

            // Elastic collision
            p.Velocity = p.Velocity - (1 + p.Bouncing) * Vector3.Dot(p.Velocity, N) * N;

            // Apply friction
            p.Velocity = p.Velocity - friction * velocityTangent;
        }

        public static bool IsCrossingPlane(Particle p, Vector3 N, float d)
        {
            float sign = Vector3.Dot(p.Position, N) + d;
            sign *= Vector3.Dot(p.PreviousPosition, N) + d;
            return sign <= 0;
        }

        public static bool IsNegativeSpacePlane(Particle p, Vector3 N, float d)
        {
            float sign = Vector3.Dot(p.Position, N) + d;
            return sign <= 0;
        }
    }
}