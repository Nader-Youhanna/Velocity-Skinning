#pragma once

#include "cgp/cgp.hpp"
#include "../skeleton/skeleton.hpp"

struct rigged_model_structure {
    cgp::mesh mesh_bind_pose;  // Bind pose (/un-deformed) mesh
    cgp::mesh mesh_deformed;   // Deformed mesh
    cgp::numarray<cgp::numarray<float> > skinning_weight; //skinning_weight[k_vertex][k_joint]
    cgp::numarray<cgp::numarray<float>> velocity_skinning_weight;
    cgp::numarray<cgp::numarray<cgp::vec3>> linear_velocities;
    cgp::numarray<cgp::numarray<cgp::vec3>> rotational_velocities;
};


struct animated_model_structure {
    rigged_model_structure rigged_mesh;
    skeleton_structure skeleton;
    double k_floppy = 1;

    // Compute the Linear Blend Skinning deformation
    //  Once computed, the rigged_mesh contains the updated deformed meshes
    void skinning_lbs();

    // Compute Dual Quaternion Skinning deformation
    //  Once computed, the rigged_mesh should contain the updated deformed meshes
    void skinning_dqs();

    void compute_rotational_velocities();

    void apply_floppy_transform(numarray<numarray<vec3>>& linear_velocities, numarray<numarray<vec3>>& rotational_velocities);
};