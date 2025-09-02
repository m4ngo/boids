### Boids: practicing using Unity DOTS
- This repository is a simple project I created to learn Unity DOTS.
- I combined ECS, Burst, and Jobs together to create a fairly optimized boid simulation.
- The result was over 200,000 boids running simultaneously!

### Process
- I started with a very naive approach. Each boid would search through every other existing boid to find the ones within its sensing range. This approach was extremely expensive and only allowed for a few thousand boids.
- I tried using jobs to parallelize this computation, making it marginally faster. (IIRC) I squeaked out about 10,000 boids on my low-end gaming laptop.
- Finally, I made the shift and used a position hashing function to store each boid's position into a 'cell.' This allows boids to lookup the nearby boids by cell, instead of searching through every single other boid. This improved the efficiency enormously. With the addition of Unity's burst compilation, I was able to simulate over 200,000 boids.
