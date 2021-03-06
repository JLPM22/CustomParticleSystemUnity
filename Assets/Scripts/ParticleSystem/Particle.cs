using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    [System.Serializable]
    public class Particle
    {
        public static float Mass;
        public static float Radius;
        public static float Bouncing;

        public Vector3 Position { get; private set; }
        public Vector3 Velocity { get; private set; }
        public Vector3 PreviousPosition { get; private set; }
        public Vector3 PreviousVelocity { get; private set; }
        public float LifeTime { get; private set; }

        private Vector3 Force;

        public void Init(Vector3 position, Vector3 velocity, Vector3 force, float lifeTime, float deltaTime)
        {
            Position = position;
            PreviousPosition = Position - velocity * deltaTime;
            Velocity = velocity;
            PreviousVelocity = velocity;
            Force = force;
            LifeTime = lifeTime;
        }

        public void UpdateEulerOrig(float deltaTime)
        {
            if (LifeTime > 0.0f)
            {
                PreviousPosition = Position;
                PreviousVelocity = Velocity;
                Position += Velocity * deltaTime;
                Velocity += (Force / Mass) * deltaTime;
                UpdateLifeTime(deltaTime);
            }
        }

        public void UpdateEulerSemi(float deltaTime)
        {
            if (LifeTime > 0.0f)
            {
                PreviousPosition = Position;
                PreviousVelocity = Velocity;
                Velocity += (Force / Mass) * deltaTime;
                Position += Velocity * deltaTime;
                UpdateLifeTime(deltaTime);
            }
        }

        public void UpdateVerlet(float deltaTime, float k)
        {
            if (LifeTime > 0.0f)
            {
                Vector3 previousPreviousPosition = PreviousPosition;
                PreviousPosition = Position;
                PreviousVelocity = Velocity;
                Position = PreviousPosition + k * (PreviousPosition - previousPreviousPosition) + ((deltaTime * deltaTime) * Force) / Mass;
                Velocity = (Position - PreviousPosition) / deltaTime;
                UpdateLifeTime(deltaTime);
            }
        }

        public void SetPosition(Vector3 newPos)
        {
            Position = newPos;
        }

        public void SetPreviousPosition(float deltaTime)
        {
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

        private void UpdateLifeTime(float deltaTime)
        {
            LifeTime -= deltaTime;
            if (LifeTime < 0.0f) LifeTime = 0.0f;
        }
    }

    public enum Solver
    {
        EulerOrig,
        EulerSemi,
        Verlet
    }
}