#pragma once

#include "cgp/cgp.hpp"
#include "../skeleton/skeleton.hpp"

using namespace cgp;

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
    float k_floppy = 0.5f;

    // Compute the Linear Blend Skinning deformation
    //  Once computed, the rigged_mesh contains the updated deformed meshes
    void skinning_lbs();
    void velocity_skinning();

    // Compute Dual Quaternion Skinning deformation
    //  Once computed, the rigged_mesh should contain the updated deformed meshes
    void skinning_dqs();

    void print_matrix4(cgp::mat4 mat);
    void print_matrix3(cgp::mat3 mat);
    void apply_floppy_transform(cgp::numarray<cgp::vec3>& result_transform, cgp::vec3 rotation_axis);
    void compute_velocities(bool first_frame);
};