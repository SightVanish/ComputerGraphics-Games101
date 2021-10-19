#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
       Vector2D interDistance = (end-start)/(num_nodes+1);
       for (int i=0; i<num_nodes+2; i++)
       {
           Mass *node = new Mass(start+interDistance*i, node_mass, false);
           masses.push_back(node);
           if(i!=0)
           {
               Spring *spring=new Spring(masses[i-1], masses[i], k);
               springs.push_back(spring);
           }
       }

        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D a2b = s->m2->position - s->m1->position;
            Vector2D f_b = -s->k*a2b/a2b.norm()*(a2b.norm()-s->rest_length);
            s->m2->forces += f_b;
            Vector2D f_a =-f_b;
            s->m1->forces += f_a;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity*m->mass;

                m->forces -= 0.025*m->velocity; // TODO (Part 2): Add global damping

                Vector2D a = m->forces / m->mass; // a=f/m
                // semi-implicit method
                m->velocity += a*delta_t;
                m->position += m->velocity*delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D a2b = s->m2->position - s->m1->position;
            Vector2D f_b = -s->k*a2b/a2b.norm()*(a2b.norm()-s->rest_length);
            s->m2->forces += f_b;
            Vector2D f_a =-f_b;
            s->m1->forces += f_a;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity*m->mass;
                Vector2D a = m->forces / m->mass; // a=f/m
                // TODO (Part 4): Add global Verlet damping
                // x(t+1)=x(t)+(1-damping_factor)*[x[t]-x[t-1]]+a(t)*dt*dt
                m->position += (1-0.00003)*(m->position-m->last_position)+a*delta_t*delta_t;
            }
        }
    }
}
