using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    [System.Serializable]
    public class Particle
    {
        public Vector3 Position { get; private set; }
        public Vector3 Velocity { get; private set; }
        public Vector3 PreviousPosition { get; private set; }
        public Vector3 PreviousVelocity { get; private set; }
        public float Bouncing { get; private set; }

        private Vector3 Force;
        private float Mass;
        private float Radius;

        public void Init(Vector3 position, Vector3 velocity, Vector3 force, float mass, float bouncing, float radius, float deltaTime)
        {
            Position = position;
            PreviousPosition = Position - velocity * deltaTime;
            Velocity = velocity;
            PreviousVelocity = velocity;
            Force = force;
            Mass = mass;
            Bouncing = bouncing;
            Radius = radius;
        }

        public void UpdateEulerOrig(float deltaTime)
        {
            PreviousPosition = Position;
            PreviousVelocity = Velocity;
            Position += Velocity * deltaTime;
            Velocity += (Force / Mass) * deltaTime;
        }

        public void UpdateEulerSemi(float deltaTime)
        {
            PreviousPosition = Position;
            PreviousVelocity = Velocity;
            Velocity += (Force / Mass) * deltaTime;
            Position += Velocity * deltaTime;
        }

        public void UpdateVerlet(float deltaTime, float k)
        {
            Vector3 previousPreviousPosition = PreviousPosition;
            PreviousPosition = Position;
            Position = PreviousPosition + k * (PreviousPosition - previousPreviousPosition) + ((deltaTime * deltaTime) * Force) / Mass;
            Velocity = (Position - PreviousPosition) / deltaTime;
        }

        public void SetPosition(Vector3 newPos, float deltaTime)
        {
            Position = newPos;
            PreviousPosition = Position - Velocity * deltaTime;
        }

        public void SetVelocity(Vector3 newVel)
        {
            Velocity = newVel;
        }

        public Vector3 GetBoundary(Vector3 center, Vector3 normal)
        {
            return center + normal * Radius;
        }
    }

    public enum Solver
    {
        EulerOrig,
        EulerSemi,
        Verlet
    }
}