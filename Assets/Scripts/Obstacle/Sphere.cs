using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class Sphere : Obstacle
    {
        private Vector3 Center;
        private float Radius;

        private void Update()
        {
            Center = transform.position;
            Radius = transform.localScale.x * 0.5f;
        }

        public override bool HasCollisionParticle(Particle p)
        {
            return Vector3.SqrMagnitude(p.GetBoundary(p.Position, (Center - p.Position).normalized) - Center) < Radius * Radius;
        }

        public override void CorrectCollisionParticle(Particle p, float deltaTime)
        {
            Vector3 previousPosition = p.GetBoundary(p.PreviousPosition, (Center - p.PreviousPosition).normalized);
            // Segment-Sphere intersection
            float alpha = Vector3.Dot(p.PreviousVelocity, p.PreviousVelocity);
            float beta = Vector3.Dot(2 * p.PreviousVelocity, (previousPosition - Center));
            float gamma = Vector3.Dot(Center, Center) + Vector3.Dot(previousPosition, previousPosition) - Vector3.Dot(Center, 2 * previousPosition) - Radius * Radius;
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
            Vector3 P = previousPosition + s * p.PreviousVelocity;
            // Define tangent plane on P
            Vector3 N = Vector3.Normalize(P - Center);
            // Apply collision plane-particle
            Plane.CollisionPlaneParticle(p, N, -Vector3.Dot(N, P), Friction, deltaTime);
        }
    }
}
