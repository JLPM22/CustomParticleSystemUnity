using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class Particle
    {
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 Force { get; private set; }
        public Vector3 PreviousPosition { get; private set; }
        public Vector3 PreviousVelocity { get; private set; }
        public float Mass { get; private set; }
        public float Bouncing { get; private set; }

        public void Init(Vector3 position, Vector3 velocity, Vector3 force, float mass, float bouncing)
        {
            Position = position;
            PreviousPosition = position;
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
            Velocity += Force * deltaTime;
        }

        public void UpdateEulerSemi(float deltaTime)
        {
            PreviousPosition = Position;
            PreviousVelocity = Velocity;
            Velocity += Force * deltaTime;
            Position += Velocity * deltaTime;
        }

        public void UpdateVerlet(float deltaTime)
        {
            // TODO: Implement Verlet integration
        }
    }

    public enum Solver
    {
        EulerOrig,
        EulerSemi,
        Verlet
    }
}