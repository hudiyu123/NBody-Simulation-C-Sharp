using System.Numerics;
using System;

namespace NBodySimulation
{
    public class NBodySimulator
    {
        public static int Main(string[] args)
        {
            var numParticles = 300;
            var G = 0.001f;
            var softening = 0.001f;
            var openingAngle = 0.5f;
            var timeStep = 0.05f;
            var steps = 100;

            {
                var (particles, indices) = ParticleGenerator.Generate(numParticles);
                Console.WriteLine(particles[0].Position);

                var gravitySolver = new DirectGravitySolver(G: G, softening: softening);
                var integrator = new LeapfrogIntegrator(timeStep: timeStep);
                for (int i = 0; i < steps; i++)
                {
                    gravitySolver.ComputeAcceleration(particles, indices);
                    integrator.Forward(particles, indices);
                }

                Console.WriteLine(particles[0].Position);
            }

            {
                var (particles, indices) = ParticleGenerator.Generate(numParticles);
                Console.WriteLine(particles[0].Position);

                var gravitySolver = new BarnesHutGravitySolver(G: G, softening: softening, openingAngle: openingAngle,
                    width: 2.0f, center: Vector3.Zero);
                var integrator = new LeapfrogIntegrator(timeStep: timeStep);
                for (int i = 0; i < steps; i++)
                {
                    gravitySolver.ComputeAcceleration(particles, indices);
                    integrator.Forward(particles, indices);
                }

                Console.WriteLine(particles[0].Position);
            }

            return 0;
        }
    }
}
