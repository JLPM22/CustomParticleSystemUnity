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
        public Vector3 Force { get; private set; }
        public Vector3 PreviousPosition { get; private set; }
        public Vector3 PreviousPreviousPosition { get; private set; }
        public Vector3 PreviousVelocity { get; private set; }
        public float Mass { get; private set; }
        public float Bouncing { get; private set; }

        public void Init(Vector3 position, Vector3 velocity, Vector3 force, float mass, float bouncing, float deltaTime)
        {
            Position = position;
            PreviousPosition = Position - velocity * deltaTime;
            PreviousPreviousPosition = PreviousPosition - velocity * deltaTime;
            Velocity = velocity;
            PreviousVelocity = velocity;
            Force = force;
            Mass = mass;
            Bouncing = bouncing;
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
            PreviousPreviousPosition = PreviousPosition;
            PreviousPosition = Position;
            Position = PreviousPosition + k * (PreviousPosition - PreviousPreviousPosition) + ((deltaTime * deltaTime) * Force) / Mass;
            Velocity = (Position - PreviousPosition) / deltaTime;
        }

        public void SetPosition(Vector3 newPos, float deltaTime)
        {
            Position = newPos;
            PreviousPosition = newPos - Velocity * deltaTime;
        }

        public void SetVelocity(Vector3 newVel)
        {
            Velocity = newVel;
        }
    }

    public enum Solver
    {
        EulerOrig,
        EulerSemi,
        Verlet
    }
}