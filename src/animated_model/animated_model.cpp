// Nader Khalil

#include "animated_model.hpp"


using namespace cgp;



void animated_model_structure::skinning_lbs()
{
    // ************************************************************** //
	// TO DO: Compute the Linear Blend Skinning (LBS) deformation
	// ...
	// ************************************************************** //
    //
    // Help:
    //     - The function should update the values of rigged_mesh.mesh_deformed.position based on the skeleton and bind pose (rigged_mesh.mesh_bind_pose.position)
    //     - Once the computation is working on the position, you may also update the normals
    //     - The skinning weights are available via: rigged_mesh.skinning_weight
    //       They are stored per vertex and per joint: float weight_ij = rigged_mesh.skinning_weight[vertex_i][joint_j];
    //       
    //     - Given a mat4 M representing a rigid transformation (with rotation and translation only), you can compute efficiently its inverse using the syntax
    //        mat4 M_inversed = M.inverse_assuming_rigid_transform();
    //     - Consider a mat4 M representing a projective (or affine, or rigid) transformation, and a vec3 p. We call q the transformation of p by M.
    //         - If p represents a 3D point, then q can be expressed as
    //             vec3 q = vec3( M * vec4(p,1.0f) ); or similarily vec3 q = M.transform_position(p);
    //         - If p represents a 3D vector, then q can be expressed as
    //             vec3 q = vec3( M * vec4(p,0.0f) ); or similarily vec3 q = M.transform_vector(p);
    //  

    // Example of looping over the positions of the mesh

    int N_vertex = rigged_mesh.mesh_bind_pose.position.size();
    for (int kv = 0; kv < N_vertex; ++kv)
    {

        mat4 M;
        for (int k = 0; k < rigged_mesh.skinning_weight[kv].size(); ++k)
        {
            float w = rigged_mesh.skinning_weight[kv][k];
            M += w * skeleton.joint_matrix_global[k] * skeleton.joint_matrix_global_bind_pose[k].inverse_assuming_rigid_transform();
        }

        rigged_mesh.mesh_deformed.position[kv] = M.transform_position(rigged_mesh.mesh_bind_pose.position[kv]);
        rigged_mesh.mesh_deformed.normal[kv] = M.transform_vector(rigged_mesh.mesh_bind_pose.normal[kv]);
    }
}

class dual_quat
{
public:
    dual_quat()
    {
        this->q = quaternion(0., 0., 0., 0.);
        this->q_eps = quaternion(0., 0., 0., 0.);
    }

    dual_quat(const quaternion& q0, const vec3& t)
    {
        this->q = q0;
        quaternion qt(t, 0.);
        this->q_eps = (qt * q0) / 2.;
    }

    dual_quat(const mat4& T)
    {
        affine_rt a = affine_rt::from_matrix(T);
        rotation_transform rot = a.rotation;
        quaternion q_rot = rot.get_quaternion();
        vec3 translation = a.translation;
        *this = dual_quat(q_rot, translation);
    }

    void extract_rotation_translation(quaternion& q0, vec3& t)
    {
        q0 = this->q;
        quaternion qt = 2 * this->q_eps * conjugate(this->q);
        t = qt.xyz();
    }

    // Normalizes in place
    void normalize()
    {
        this->q = cgp::normalize(this->q);
        this->q_eps = this->q_eps / cgp::norm(this->q);
    }

    dual_quat operator+(const dual_quat other) const
    {
        dual_quat res;
        res.q = this->q + other.q;
        res.q_eps = this->q_eps + other.q_eps;
        return res;
    }

    dual_quat operator-(const dual_quat other) const
    {
        dual_quat res;
        res.q = this->q - other.q;
        res.q_eps = this->q_eps - other.q_eps;
        return res;
    }

    dual_quat operator*(const dual_quat other) const
    {
        dual_quat res;
        res.q = this->q * other.q;
        res.q_eps = (this->q_eps * other.q) + (this->q * other.q_eps);
        return res;
    }

    dual_quat operator*(float x)
    {
        dual_quat res;
        res.q = this->q * x;
        res.q_eps = this->q_eps * x;
        return res;
    }

    dual_quat operator/(const dual_quat other) const
    {
        dual_quat res;
        res.q = this->q * other.q;
        res.q_eps = (this->q_eps * other.q) - (this->q * other.q_eps);
        res.q_eps /= other.q * other.q;
        return res;
    }

    void operator+=(const dual_quat other)
    {
        *this = *this + other;
    }

    void print() const
    {
        std::cout << "q: " << q << "\n";
        std::cout << "q_eps: " << q_eps << "\n";
    }

    quaternion q;
    quaternion q_eps;
//private:
};


void animated_model_structure::skinning_dqs()
{
    // ************************************************************** //
	// TO DO: Compute Dual Quaternion Skinning (DQS) deformation
	// ...
	// ************************************************************** //
    //
    // Help:
    //     - Given a mat4 representing a rigid transformation, the following syntax allows to access the rotation and translation part:
    //         affine_rt a = affine_rt::from_matrix({mat4});
    //         rotation_transform rot = a.rotation
    //         vec3 translation = a.translation
    //     - The quaternion of a rotation_transform can be accessed via {rotation_transform}.get_quaternion();
    //     - The structure quaternion is a specialized type derived from a vec4. You can access to its .x .y .z .w component similarily to a vec4.
    //     
    
    // Define number of vertices
    int N_vertex = rigged_mesh.mesh_bind_pose.position.size();
    for (int k_vertex = 0; k_vertex < N_vertex; ++k_vertex) {
        // For each vertex
        vec3 const& position_in_bind_pose = rigged_mesh.mesh_bind_pose.position[k_vertex]; // The "initial/bind pose" position p0
        vec3 const& normal_in_bind_pose = rigged_mesh.mesh_bind_pose.normal[k_vertex];     // The "initial/bind pose" normal n0
        vec3& position_to_be_deformed = rigged_mesh.mesh_deformed.position[k_vertex];      // The position to be deformed by LBS
        vec3& normal_to_be_deformed = rigged_mesh.mesh_deformed.normal[k_vertex];         // The normal to be deformed by LBS
        
        dual_quat transform_q;
        for (int j = 0; j < skeleton.size(); j++)
        {
            float w_ij = rigged_mesh.skinning_weight[k_vertex][j];
            cgp::mat4 Tj = skeleton.joint_matrix_global[j] * skeleton.joint_matrix_global_bind_pose[j].inverse_assuming_rigid_transform();
            dual_quat qi(Tj);
            transform_q += qi * w_ij;
        }
    
        transform_q.normalize();
        quaternion rot;
        vec3 t;
        transform_q.extract_rotation_translation(rot, t);
        
        quaternion v(position_in_bind_pose, 0.);
        position_to_be_deformed = (rot * v * conjugate(rot)).xyz();
        position_to_be_deformed += t;

        quaternion n(normal_in_bind_pose, 0.);
        normal_to_be_deformed = (rot * n * conjugate(rot)).xyz();
        normal_to_be_deformed += t;
    }
}

void animated_model_structure::compute_rotational_velocities()
{
    int N_vertex = rigged_mesh.mesh_bind_pose.position.size();
    int N_joint = skeleton.size();
    for (int kv = 0; kv < N_vertex; ++kv)
    {
        cgp::vec3 pi = rigged_mesh.mesh_deformed.position[kv];
        for (int kj = 0; kj < N_joint; ++kj)
        {
            cgp::vec3 angular_velocity = skeleton.angular_velocities[kj];
            cgp::vec3 pu = skeleton.joint_matrix_global_bind_pose[kj].get_block_translation();            
            rigged_mesh.rotational_velocities[kv][kj] = cgp::cross(angular_velocity, (pu - pi));
        }
    }
}