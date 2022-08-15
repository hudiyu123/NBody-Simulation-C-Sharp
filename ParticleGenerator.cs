using System;
using System.Numerics;

namespace NBodySimulation
{
    public class ParticleGenerator
    {
        static public (List<Particle> particles, List<Int32> indices) Generate(Int32 size)
        {
            var particles = new List<Particle>();
            var indices = new List<Int32>();
            var random = new Random(0);
            for (Int32 i = 0; i < size; i++)
            {
                var particle = new Particle();
                particle.Mass = random.NextSingle();
                particle.Position = GetRandomVector(random);
                particle.Velocity = Vector3.Zero;
                particle.Acceloration = Vector3.Zero;
                particles.Add(particle);

                indices.Add(i);
            }

            return (particles, indices);
        }

        static private Vector3 GetRandomVector(Random rand)
        {
            return new()
            {
                X = rand.NextSingle(),
                Y = rand.NextSingle(),
                Z = rand.NextSingle()
            };
        }
    }
}

