using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class Sphere : Obstacle
    {
        private Vector3 Center
        {
            get { return transform.position; }
        }
        private float Radius
        {
            get { return transform.localScale.x * 0.5f; }
        }

        public override bool HasCollisionParticle(Particle p)
        {
            return Vector3.SqrMagnitude(p.Position - Center) < Radius * Radius;
        }

        public override void CorrectCollisionParticle(Particle p, float deltaTime)
        {
            // Segment-Sphere intersection
            float alpha = Vector3.Dot(p.PreviousVelocity, p.PreviousVelocity);
            float beta = Vector3.Dot(2 * p.PreviousVelocity, (p.PreviousPosition - Center));
            float gamma = Vector3.Dot(Center, Center) + Vector3.Dot(p.PreviousPosition, p.PreviousPosition) - Vector3.Dot(Center, 2 * p.PreviousPosition) - Radius * Radius;
            // Solve Second order equation
            float num = beta * beta - 4 * alpha * gamma;
            if (num < 0)
            {
                // No intersection
                return;
            }
            float sqrt = Mathf.Sqrt(num);
            float s1 = (-beta + sqrt) / (2 * alpha);
            float s2 = (-beta - sqrt) / (2 * alpha);
            float s = s1 >= 0 && s1 <= deltaTime ? s1 : s2;
            // Intersection point with the sphere
            Vector3 P = p.PreviousPosition + s * p.PreviousVelocity;
            // Define tangent plane on P
            Vector3 N = Vector3.Normalize(P - Center);
            // Apply collision plane-particle
            CollisionPlaneParticle(p, N, P);
        }

        private void CollisionPlaneParticle(Particle p, Vector3 N, Vector3 P)
        {
            float D = -Vector3.Dot(N, P);

            p.Position = p.Position - (1 + p.Bouncing) * (Vector3.Dot(p.Position, N) + D) * N;

            // Friction
            Vector3 velocityNormal = Vector3.Dot(p.Velocity, N) * N;
            Vector3 velocityTangent = p.Velocity - velocityNormal;

            // Elastic collision
            p.Velocity = p.Velocity - (1 + p.Bouncing) * Vector3.Dot(p.Velocity, N) * N;
        }
    }
}
