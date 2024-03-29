// Nader Khalil

#include "animated_model.hpp"


using namespace cgp;

void animated_model_structure::print_matrix4(cgp::mat4 mat)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            std::cout << mat[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "--------------------------------------------\n";
}

void animated_model_structure::print_matrix3(cgp::mat3 mat)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::cout << mat[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "--------------------------------------------\n";
}

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

void animated_model_structure::velocity_skinning()
{
    int N_vertex = rigged_mesh.mesh_bind_pose.position.size();
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
    for (int k_vertex = 0; k_vertex < N_vertex; ++k_vertex)
    {
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


void animated_model_structure::apply_floppy_transform(cgp::numarray<cgp::vec3>& result_transform, cgp::vec3 rotation_axis)
{
    int N_vertex = rigged_mesh.mesh_bind_pose.position.size();
    int N_joint = skeleton.size();

    for (int kv = 0; kv < N_vertex; kv++)
    {
        cgp::vec3 psi = cgp::vec3(0.0, 0.0, 0.0);
        for (int kj = 0; kj < N_joint; kj++)
        {
            // Compute linear transformation
            cgp::vec3 psi_linear = -k_floppy * rigged_mesh.linear_velocities[kv][kj];
            
            // Compute rotational transformation
            double rotation_angle = -k_floppy * cgp::norm(rigged_mesh.rotational_velocities[kv][kj]);
            rotation_transform R = rotation_axis_angle(normalize(rotation_axis), rotation_angle);
            mat3 identity_matrix = mat3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
            cgp::vec3 pu = skeleton.joint_matrix_global_bind_pose[kj].get_block_translation();
            cgp::vec3 pu_rot_projection = R * pu;
            cgp::vec3 psi_rotational = vec3(0.0, 0.0, 0.0); //TODO: Add actual computation
            //cgp::vec3 psi_rotational = (R - identity_matrix) * (pu - pu_rot_projection);
            
            // Compute final transformation
            psi += psi_linear + psi_rotational;
        }
        result_transform[kv] = psi;
    }
}

void animated_model_structure::compute_velocities(bool first_frame)
{
    int N_vertex = rigged_mesh.mesh_bind_pose.position.size();
    int N_joint = skeleton.size();
    // Resize and clear matrices
    rigged_mesh.linear_velocities.resize_clear(N_vertex);
    rigged_mesh.rotational_velocities.resize_clear(N_vertex);
    for (int kv = 0; kv < N_vertex; ++kv)
    {
        rigged_mesh.linear_velocities[kv].resize_clear(N_joint);
        rigged_mesh.rotational_velocities[kv].resize_clear(N_joint);
    }
    ////-----------------------------------------------------------------////

    // If not first frame, then last frame matrices exist
    if (!first_frame)
    {
        // for each joint kj
        for (int kj = 0; kj < N_joint; kj++)
        {
            // Get the local transformation matrix T_j
            mat4 Tj_local = skeleton.joint_matrix_local[kj]
                * skeleton.joint_matrix_local_last_frame[kj]
                .inverse_assuming_rigid_transform();

            // Get the parent matrix M_parent
            mat4 M_parent;
            if (kj == 0)
            {
                M_parent = skeleton.joint_matrix_local[0];
            }
            else
            {
                mat4 M_parent = skeleton.joint_matrix_global[skeleton.parent_index[kj]];
            }

            // Set the parent matrix translation to zero
            M_parent.set_block_translation(vec3(0.f, 0.f, 0.f));

            // Get the global transformation matrix
            //mat4 Tj_global = M_parent * Tj_local;
            mat4 Tj_global = Tj_local;

            // Extract the translation
            vec3 translation = Tj_global.get_block_translation();

            // Extract the rotation
            mat3 R_mat = Tj_global.get_block_linear();
            //print_matrix3(R_mat);
            rotation_transform rot = rotation_transform::from_matrix(R_mat);
            vec3 rot_axis; float angle;
            rot.to_axis_angle(rot_axis, angle);
            vec3 angular_velocity = cgp::normalize(rot_axis) * angle;
            std::cout << "Rotational velocity: " << angular_velocity << std::endl;

            // for each vertex kv
            for (int kv = 0; kv < N_vertex; kv++)
            {

                // Set the linear velocities
                rigged_mesh.linear_velocities[kv][kj] = translation;

                // Set the rotational velocities
                //vec3 pi = skeleton.joint_matrix_global_bind_pose[kj].get_block_translation();
                //vec3 pu = rigged_mesh.mesh_deformed.position[kv];
                //rigged_mesh.rotational_velocities[kv][kj] = cgp::cross(angular_velocity, (pu - pi));
                rigged_mesh.rotational_velocities[kv][kj] = angular_velocity;

            }
        }
        int kv = 2400;
        int kj = 1;
    }

}